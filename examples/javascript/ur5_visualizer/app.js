import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
// import createKinexModule from '../../../build-wasm/wasm/kinex.js'; // Loaded via script tag

// Configuration
const URDF_PATH = '../../../examples/models/ur5/ur5e.urdf';
const MESH_BASE_PATH = '../../../examples/models/ur5/'; // Meshes are relative to this
const END_EFFECTOR_LINK = 'wrist_3_link';

// Globals
let scene, camera, renderer, controls, transformControl;
let kinex, robot, solver, fk;
let robotVisuals = {}; // Map<linkName, THREE.Group>
let currentJoints; // Float64Array
let targetSphere;
let isDragging = false;

// Expose for debugging
window.debug = {
    get scene() { return scene; },
    get robot() { return robot; },
    get solver() { return solver; },
    get currentJoints() { return currentJoints; },
    get targetSphere() { return targetSphere; },
    get fk() { return fk; },
    updateRobotPose: updateRobotPose,
    onTargetChange: onTargetChange
};

async function init() {
    // 1. Setup Three.js
    setupThreeJS();

    // 2. Load Kinex
    try {
        kinex = await createKinexModule({
            locateFile: (path, prefix) => {
                if (path.endsWith('.wasm')) {
                    return '../../../build-wasm/wasm/' + path;
                }
                return prefix + path;
            }
        });
        console.log("Kinex loaded");
    } catch (e) {
        console.error("Failed to load Kinex:", e);
        document.getElementById('loading').innerText = "Failed to load Kinex: " + e;
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
        robot = kinex.Robot.fromURDFString(urdfContent, "");
        const dof = robot.getDOF();
        console.log(`Robot loaded: ${robot.getName()}, DOF: ${dof}`);
        
        // Initialize joints to 0 (or a home position)
        currentJoints = new Float64Array(dof);
        // Set some initial angles to avoid singularity if needed, or just 0
        // For UR5, maybe a bit of a pose
        if (dof >= 6) {
            currentJoints[1] = -1.57;
            currentJoints[2] = 1.57;
            currentJoints[3] = -1.57;
            currentJoints[4] = -1.57;
        }

        // Initialize Solvers
        console.log("Initializing SQPIKSolver...");
        solver = new kinex.SQPIKSolver(robot, END_EFFECTOR_LINK, "");
        console.log("SQPIKSolver initialized");

        console.log("Initializing ForwardKinematics...");
        fk = new kinex.ForwardKinematics(robot, END_EFFECTOR_LINK, "");
        console.log("ForwardKinematics initialized");
        
        // Configure Solver
        const config = solver.getConfig();
        config.max_iterations = 100;
        config.tolerance = 1e-4;
        solver.setConfig(config);

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
    scene.background = new THREE.Color(0x222222);
    
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
    const ambientLight = new THREE.AmbientLight(0x404040);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1);
    dirLight.position.set(1, 1, 2);
    dirLight.castShadow = true;
    scene.add(dirLight);

    // Grid
    const gridHelper = new THREE.GridHelper(10, 10);
    gridHelper.rotation.x = Math.PI / 2; // Rotate to be on XY plane
    scene.add(gridHelper);

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
    const links = robot.getLinks();
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
            
            // Only handle meshes for now
            if (geometry.type === kinex.GeometryType.Mesh) {
                const meshFilename = geometry.mesh_filename;
                const scale = geometry.mesh_scale;
                
                // Construct path
                const meshPath = MESH_BASE_PATH + meshFilename;
                
                try {
                    const object = await loadMesh(loader, meshPath);
                    
                    // Apply scale
                    object.scale.set(scale[0], scale[1], scale[2]);
                    
                    // Apply visual origin
                    const visualGroup = new THREE.Group();
                    visualGroup.add(object);
                    
                    const pos = origin.translation();
                    const quat = origin.asPose().quaternion; // w, x, y, z
                    
                    visualGroup.position.set(pos[0], pos[1], pos[2]);
                    visualGroup.quaternion.set(quat[1], quat[2], quat[3], quat[0]); // Three.js is x, y, z, w
                    
                    linkGroup.add(visualGroup);
                    
                    // Material
                    object.traverse((child) => {
                        if (child.isMesh) {
                            child.material = new THREE.MeshStandardMaterial({
                                color: 0xaaaaaa,
                                roughness: 0.5,
                                metalness: 0.5
                            });
                            child.castShadow = true;
                            child.receiveShadow = true;
                        }
                    });

                } catch (e) {
                    console.warn(`Failed to load mesh ${meshPath}:`, e);
                }
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
    // Target Sphere
    const geometry = new THREE.SphereGeometry(0.05, 32, 32);
    const material = new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.5 });
    targetSphere = new THREE.Mesh(geometry, material);
    scene.add(targetSphere);

    // Transform Controls
    transformControl = new TransformControls(camera, renderer.domElement);
    transformControl.addEventListener('dragging-changed', function (event) {
        controls.enabled = !event.value;
        isDragging = event.value;
    });
    transformControl.addEventListener('change', onTargetChange);
    
    transformControl.attach(targetSphere);
    scene.add(transformControl);

    // Keyboard controls for mode switching
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

    // Initial position of target (match end effector)
    const eePose = fk.compute(currentJoints);
    targetSphere.position.set(eePose.position[0], eePose.position[1], eePose.position[2]);
    // Quaternion
    const q = eePose.quaternion;
    targetSphere.quaternion.set(q[1], q[2], q[3], q[0]);
}

function onTargetChange() {
    // Called when user drags the sphere
    // Perform IK
    
    const targetPos = targetSphere.position;
    const targetQuat = targetSphere.quaternion;

    console.log("--- IK Target (Three.js Frame) ---");
    console.log(`Pos: [${targetPos.x.toFixed(4)}, ${targetPos.y.toFixed(4)}, ${targetPos.z.toFixed(4)}]`);
    console.log(`Quat (xyzw): [${targetQuat.x.toFixed(4)}, ${targetQuat.y.toFixed(4)}, ${targetQuat.z.toFixed(4)}, ${targetQuat.w.toFixed(4)}]`);

    const targetPose = {
        position: [targetPos.x, targetPos.y, targetPos.z],
        quaternion: [targetQuat.w, targetQuat.x, targetQuat.y, targetQuat.z] // w, x, y, z
    };

    // Solve IK
    // Use current joints as seed
    const result = solver.solve(targetPose, currentJoints);
    
    // Update joints regardless of convergence (best effort)
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

    // Check FK of solution
    const eePose = fk.compute(currentJoints);
    console.log("--- Solved FK (Kinex Frame) ---");
    console.log(`Pos: [${eePose.position[0].toFixed(4)}, ${eePose.position[1].toFixed(4)}, ${eePose.position[2].toFixed(4)}]`);
    console.log(`Quat (wxyz): [${eePose.quaternion[0].toFixed(4)}, ${eePose.quaternion[1].toFixed(4)}, ${eePose.quaternion[2].toFixed(4)}, ${eePose.quaternion[3].toFixed(4)}]`);

    updateRobotPose();
}

function updateRobotPose() {
    // Get all link transforms in one call (much more efficient!)
    const linkTransforms = fk.computeAllLinkTransforms(currentJoints);
    
    if (linkTransforms.has('base')) {
         const baseT = linkTransforms.get('base');
         console.log(`Base Transform: Pos=[${baseT.position.join(', ')}], Quat=[${baseT.quaternion.join(', ')}]`);
    }

    // Update each link's visual pose
    linkTransforms.forEach((pose, linkName) => {
        if (robotVisuals[linkName]) {
            const pos = pose.position;
            const quat = pose.quaternion; // w, x, y, z
            
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
