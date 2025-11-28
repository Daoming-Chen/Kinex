import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
// import createKinexModule from '@daoming.chen/kinex'; // Loaded dynamically

// Configuration
const URDF_PATH = '../models/ur5/ur5e+x.urdf';
const MESH_BASE_PATH = '../models/ur5/'; // Meshes are relative to this
const END_EFFECTOR_LINK = 'wrist_3_link';

// Globals
let scene, camera, renderer, controls, transformControl;
let kinex, robot, robotModel, fk;
let robotVisuals = {}; // Map<linkName, THREE.Group>
let currentJoints; // Float64Array
let targetSphere;
let isDragging = false;

// Expose for debugging
window.debug = {
    get scene() { return scene; },
    get robot() { return robot; },
    get robotModel() { return robotModel; },
    get currentJoints() { return currentJoints; },
    get targetSphere() { return targetSphere; },
    get fk() { return fk; },
    updateRobotPose: updateRobotPose,
    onTargetChange: onTargetChange
};

async function init() {
    // 1. Setup Three.js
    setupThreeJS();

    // 2. Load Kinex from npm
    let createKinexModule;
    let locateFile = undefined;

    const loadScript = (src) => {
        return new Promise((resolve, reject) => {
            const script = document.createElement('script');
            script.src = src;
            script.onload = () => resolve();
            script.onerror = (e) => reject(e);
            document.head.appendChild(script);
        });
    };

    try {
        // Try loading from npm via import map
        const module = await import('@daoming.chen/kinex');
        createKinexModule = module.default;

        if (typeof createKinexModule !== 'function') {
            console.log("Import failed to provide function, trying script tag for npm...");
            await loadScript('https://unpkg.com/@daoming.chen/kinex@latest/kinex.js');
            createKinexModule = window.createKinexModule;
        }

        console.log("Loaded Kinex from npm");

        locateFile = (path, prefix) => {
            if (path.endsWith('.wasm')) {
                return '../../build-wasm/wasm/kinex.wasm';
            }
            return prefix + path;
        };
    } catch (e) {
        console.error("Failed to load Kinex:", e);
        document.getElementById('loading').innerText = "Failed to load Kinex: " + e;
        return;
    }

    try {
        console.log("Calling createKinexModule...");
        kinex = await createKinexModule({
            locateFile: locateFile,
            print: (text) => console.log("[Kinex stdout]: " + text),
            printErr: (text) => console.error("[Kinex stderr]: " + text),
        });
        console.log("Kinex initialized");
    } catch (e) {
        console.error("Failed to initialize Kinex:", e);
        if (typeof e === 'object') {
             console.error("Error details:", JSON.stringify(e, Object.getOwnPropertyNames(e)));
        }
        document.getElementById('loading').innerText = "Failed to initialize Kinex: " + e;
        return;
    }

    // 3. Load URDF
    let urdfContent;
    try {
        const response = await fetch(URDF_PATH);
        urdfContent = await response.text();
        console.log("URDF loaded");
    } catch (e) {
        console.error("Failed to load URDF:", e);
        document.getElementById('loading').innerText = "Failed to load URDF: " + e;
        return;
    }

    // 4. Initialize Robot
    try {
        // Create Unified Robot for kinematics
        robot = kinex.Robot.fromURDFString(urdfContent, END_EFFECTOR_LINK, "");
        
        // Create RobotModel for structure traversal and visual setup
        // (This is necessary because the unified Robot wrapper in WASM currently hides the structure details)
        robotModel = kinex.RobotModel.fromURDFString(urdfContent, "");
        
        const dof = robot.getDOF();
        console.log(`Robot loaded: ${robot.getName()}, DOF: ${dof}`);
        
        // Initialize joints
        currentJoints = new Float64Array(dof);
        if (dof >= 6) {
            currentJoints[1] = -1.57;
            currentJoints[2] = 1.57;
            currentJoints[3] = -1.57;
            currentJoints[4] = -1.57;
        }

        // Configure Unified Robot Solver
        const config = robot.getSolverConfig();
        config.max_iterations = 100;
        config.tolerance = 1e-4;
        robot.setSolverConfig(config);
        
        // Create helper FK for full-body visualization update
        // (This is necessary because the unified Robot wrapper doesn't expose computeAllLinkTransforms yet)
        fk = new kinex.ForwardKinematics(robotModel, END_EFFECTOR_LINK, "");
        
        console.log("Robot and solvers initialized");

    } catch (e) {
        console.error("Failed to initialize Robot:", e);
        document.getElementById('loading').innerText = "Failed to initialize Robot: " + e;
        return;
    }

    // 5. Create Visuals
    await createRobotVisuals();

    // 6. Setup Interaction
    setupInteraction();

    // 7. Initial Update
    updateRobotPose();

    // Hide loading
    document.getElementById('loading').style.display = 'none';

    // 8. Start Loop
    animate();
}

function setupThreeJS() {
    const container = document.getElementById('container');
    
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x2e8b57);
    
    // Camera
    camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.position.set(2, 2, 2);
    camera.up.set(0, 0, 1); // Z-up
    camera.lookAt(0, 0, 0);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xaaaaaa);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
    dirLight.position.set(1, 1, 2);
    dirLight.castShadow = true;
    scene.add(dirLight);

    const axesHelper = new THREE.AxesHelper(1);
    scene.add(axesHelper);

    // Controls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Resize handler
    window.addEventListener('resize', onWindowResize);
}

async function createRobotVisuals() {
    // Use robotModel for structure traversal
    const links = robotModel.getLinks();
    const numLinks = links.size();
    const loader = new OBJLoader();

    for (let i = 0; i < numLinks; i++) {
        const link = links.get(i);
        const linkName = link.getName();
        const visuals = link.getVisuals();
        const numVisuals = visuals.size();

        const linkGroup = new THREE.Group();
        linkGroup.name = linkName;
        scene.add(linkGroup);
        robotVisuals[linkName] = linkGroup;

        for (let j = 0; j < numVisuals; j++) {
            const visual = visuals.get(j);
            const geometry = visual.geometry;
            const origin = visual.origin;
            
            let object = null;
            let scale = [1, 1, 1];

            // Handle different geometry types
            if (geometry.type === kinex.GeometryType.Mesh) {
                const meshFilename = geometry.mesh_filename;
                scale = geometry.mesh_scale;
                const meshPath = MESH_BASE_PATH + meshFilename;
                
                try {
                    object = await loadMesh(loader, meshPath);
                } catch (e) {
                    console.warn(`Failed to load mesh ${meshPath}:`, e);
                    continue;
                }
            } else if (geometry.type === kinex.GeometryType.Box) {
                const size = geometry.box_size;
                const geo = new THREE.BoxGeometry(size[0], size[1], size[2]);
                object = new THREE.Mesh(geo);
            } else if (geometry.type === kinex.GeometryType.Cylinder) {
                const radius = geometry.cylinder_radius;
                const length = geometry.cylinder_length;
                const geo = new THREE.CylinderGeometry(radius, radius, length, 32);
                geo.rotateX(Math.PI / 2);
                object = new THREE.Mesh(geo);
            } else if (geometry.type === kinex.GeometryType.Sphere) {
                const radius = geometry.sphere_radius;
                const geo = new THREE.SphereGeometry(radius, 32, 32);
                object = new THREE.Mesh(geo);
            }

            if (object) {
                if (geometry.type === kinex.GeometryType.Mesh) {
                    object.scale.set(scale[0], scale[1], scale[2]);
                }

                let color = 0xeeeeee;
                let opacity = 1.0;
                let transparent = false;
                
                if (visual.getColor()) {
                     const c = visual.getColor();
                     if (c.length >= 3) {
                         color = new THREE.Color(c[0], c[1], c[2]);
                     }
                     if (c.length >= 4) {
                         opacity = c[3];
                         if (opacity < 1.0) transparent = true;
                     }
                }

                const material = new THREE.MeshStandardMaterial({
                    color: color,
                    roughness: 0.4,
                    metalness: 0.6,
                    transparent: transparent,
                    opacity: opacity
                });

                object.traverse((child) => {
                    if (child.isMesh) {
                        child.material = material;
                        child.castShadow = true;
                        child.receiveShadow = true;
                    }
                });
                if (object.isMesh) {
                    object.material = material;
                    object.castShadow = true;
                    object.receiveShadow = true;
                }

                const visualGroup = new THREE.Group();
                visualGroup.add(object);
                
                const pos = origin.translation();
                const quat = origin.asPose().quaternion;
                
                visualGroup.position.set(pos[0], pos[1], pos[2]);
                visualGroup.quaternion.set(quat[1], quat[2], quat[3], quat[0]);
                
                linkGroup.add(visualGroup);
            }
        }
    }
}

function loadMesh(loader, url) {
    return new Promise((resolve, reject) => {
        loader.load(url, resolve, undefined, reject);
    });
}

function setupInteraction() {
    const geometry = new THREE.SphereGeometry(0.05, 32, 32);
    const material = new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.5 });
    targetSphere = new THREE.Mesh(geometry, material);
    scene.add(targetSphere);

    transformControl = new TransformControls(camera, renderer.domElement);
    transformControl.addEventListener('dragging-changed', function (event) {
        controls.enabled = !event.value;
        isDragging = event.value;
    });
    transformControl.addEventListener('change', onTargetChange);
    
    transformControl.attach(targetSphere);
    scene.add(transformControl);

    window.addEventListener('keydown', function (event) {
        switch (event.key) {
            case 't':
                transformControl.setMode('translate');
                break;
            case 'r':
                transformControl.setMode('rotate');
                break;
        }
    });

    // Use Robot API for FK
    const eePose = robot.forwardKinematics(currentJoints);
    targetSphere.position.set(eePose.position[0], eePose.position[1], eePose.position[2]);
    const q = eePose.quaternion;
    targetSphere.quaternion.set(q[1], q[2], q[3], q[0]);
}

function onTargetChange() {
    const targetPos = targetSphere.position;
    const targetQuat = targetSphere.quaternion;

    console.log("--- IK Target (Three.js Frame) ---");
    console.log(`Pos: [${targetPos.x.toFixed(4)}, ${targetPos.y.toFixed(4)}, ${targetPos.z.toFixed(4)}]`);
    console.log(`Quat (xyzw): [${targetQuat.x.toFixed(4)}, ${targetQuat.y.toFixed(4)}, ${targetQuat.z.toFixed(4)}, ${targetQuat.w.toFixed(4)}]`);

    const targetPose = {
        position: [targetPos.x, targetPos.y, targetPos.z],
        quaternion: [targetQuat.w, targetQuat.x, targetQuat.y, targetQuat.z]
    };

    // Use Robot API for IK
    const result = robot.inverseKinematics(targetPose, currentJoints);
    
    const solution = result.solution;
    const dof = robot.getDOF();
    
    if (solution.get) {
        for (let i = 0; i < dof; i++) {
            currentJoints[i] = solution.get(i);
        }
    } else {
        for (let i = 0; i < dof; i++) {
            currentJoints[i] = solution[i];
        }
    }

    // Use Robot API for FK check
    const eePose = robot.forwardKinematics(currentJoints);
    console.log("--- Solved FK (Kinex Frame) ---");
    console.log(`Pos: [${eePose.position[0].toFixed(4)}, ${eePose.position[1].toFixed(4)}, ${eePose.position[2].toFixed(4)}]`);
    console.log(`Quat (wxyz): [${eePose.quaternion[0].toFixed(4)}, ${eePose.quaternion[1].toFixed(4)}, ${eePose.quaternion[2].toFixed(4)}, ${eePose.quaternion[3].toFixed(4)}]`);

    updateRobotPose();
}

function updateRobotPose() {
    // Use helper FK for full visualization update (computes all frames efficiently)
    const linkTransforms = fk.computeAllLinkTransforms(currentJoints);
    
    if (linkTransforms.has('base')) {
         const baseT = linkTransforms.get('base');
         console.log(`Base Transform: Pos=[${baseT.position.join(', ')}], Quat=[${baseT.quaternion.join(', ')}]`);
    }

    linkTransforms.forEach((pose, linkName) => {
        if (robotVisuals[linkName]) {
            const pos = pose.position;
            const quat = pose.quaternion;
            
            robotVisuals[linkName].position.set(pos[0], pos[1], pos[2]);
            robotVisuals[linkName].quaternion.set(quat[1], quat[2], quat[3], quat[0]);
        }
    });
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

init();
