from browser import document, window, ajax, bind

THREE = window.THREE

# Shortcuts to JS constructors
PerspectiveCamera = THREE.PerspectiveCamera.new
Scene = THREE.Scene.new
WebGLRenderer = THREE.WebGLRenderer.new
MeshStandardMaterial = THREE.MeshStandardMaterial
SphereGeometry = THREE.SphereGeometry.new
Mesh = THREE.Mesh.new
AmbientLight = THREE.AmbientLight.new
DirectionalLight = THREE.DirectionalLight.new
OrbitControls = window.THREE.OrbitControls.new
CubeTextureLoader = THREE.CubeTextureLoader.new


# Globals
camera = None
renderer = None
scene = None
processed_data = []


# joints and links
joint1 = None
joint2 = None
joint3 = None
joint4 = None
link1 = None
link2 = None
link3 = None


def receive_data():
    def on_complete(req):
        global processed_data
        if req.status == 200:
            raw_data = window.JSON.parse(req.responseText)
            #print("✅ Received raw data:", req.responseText)

            processed_data = []  # Reset processed data

            for i, array in enumerate(raw_data):
                try:
                    # Convert None to NaN where needed
                    cleaned = [window.NaN if x is None else x for x in array]

                    # Avoid printing objects with potential None directly
                    #print(f" Array {i} (safe): {[str(x) for x in cleaned]}")

                    # Add to processed list
                    processed_data.append(cleaned)

                except Exception as e:
                    print(f" Error processing array {i}:", e)

            # ➕ You can now work with `processed_data` safely (e.g., visualization, plotting)
            # Example: pass to a drawing or plotting function here

        else:
            print(" Error loading data. HTTP Status:", req.status)

    req = ajax.Ajax()
    req.bind('complete', on_complete)
    req.open('GET', '/send_data', True)
    req.send()

# Call the function to get the data
receive_data()

@bind(document, "keydown")
def on_keydown(evt):
    global joint1, joint2, joint3, joint4
    global link1, link2, link3
    global processed_data
    if evt.key == " ":
        print("🟦 Spacebar pressed! Processed data:")
        for i, row in enumerate(processed_data):
            print(f"{i}: {[str(x) for x in row]}")
    if evt.key == "Enter":
        print("🟦 Enter pressed! Sending data to server...")
        # Send data to the server
        print(joint1.position.x, joint1.position.y, joint1.position.z)
        print(joint2.position.x, joint2.position.y, joint2.position.z)
        print(joint3.position.x, joint3.position.y, joint3.position.z)
        print(joint4.position.x, joint4.position.y, joint4.position.z)
        joint1.position.x = processed_data[0][0]
        joint1.position.z = processed_data[0][1]
        joint1.position.y = processed_data[0][2]
        joint2.position.x = processed_data[1][0]
        joint2.position.z = processed_data[1][1]
        joint2.position.y = processed_data[1][2]
        joint3.position.x = processed_data[2][0]
        joint3.position.z = processed_data[2][1]
        joint3.position.y = processed_data[2][2]
        joint4.position.x = processed_data[3][0]
        joint4.position.z = processed_data[3][1]
        joint4.position.y = processed_data[3][2]
        # Update the links based on the new joint positions
        update_links()

def update_links():
    """Update the links based on the current joint positions."""
    global link1, link2, link3

    # Update the links' positions and orientations
    link1.geometry.dispose()  # Dispose of the old geometry
    link1 = create_link(joint1, joint2)  # Create a new link between joint1 and joint2
    scene.add(link1)  # Add the new link to the scene

    link2.geometry.dispose()  # Dispose of the old geometry
    link2 = create_link(joint2, joint3)  # Create a new link between joint2 and joint3
    scene.add(link2)  # Add the new link to the scene

    link3.geometry.dispose()  # Dispose of the old geometry
    link3 = create_link(joint3, joint4)  # Create a new link between joint3 and joint4
    scene.add(link3)  # Add the new link to the scene

def create_joint(x, y, z):
    """Create a joint (sphere) at the specified coordinates."""
    # Create a sphere geometry for the joint
    sphere_geometry = SphereGeometry(1, 32, 32)  # Radius of 5 with 32 segments for a smooth sphere
    sphere_material = MeshStandardMaterial.new({'color': 0x00ffff})  # Corrected instantiation with new
    sphere = Mesh(sphere_geometry, sphere_material)

    # Position the sphere at the specified coordinates
    sphere.position.set(x, y, z)

    return sphere

def create_link(start_joint, end_joint):
    """Create a link (cylinder) between two joints."""
    # Vector difference
    dx = end_joint.position.x - start_joint.position.x
    dy = end_joint.position.y - start_joint.position.y
    dz = end_joint.position.z - start_joint.position.z
    length = window.Math.sqrt(dx * dx + dy * dy + dz * dz)

    # Cylinder geometry along Y-axis
    cylinder_geometry = THREE.CylinderGeometry.new(1, 1, length, 32)
    cylinder_material = MeshStandardMaterial.new({'color': 0x00ff00})
    cylinder = Mesh(cylinder_geometry, cylinder_material)

    # Midpoint between start and end
    cylinder.position.set(
        (start_joint.position.x + end_joint.position.x) / 2,
        (start_joint.position.y + end_joint.position.y) / 2,
        (start_joint.position.z + end_joint.position.z) / 2,
    )

    # Direction from start to end
    direction = window.THREE.Vector3.new(dx, dy, dz).normalize()

    # Align cylinder's Y-axis to direction vector
    up = window.THREE.Vector3.new(0, 1, 0)  # Y-axis
    quaternion = window.THREE.Quaternion.new()
    quaternion.setFromUnitVectors(up, direction)
    cylinder.setRotationFromQuaternion(quaternion)

    return cylinder
def init():
    global camera, renderer, scene
    global joint1, joint2, joint3, joint4
    global link1, link2, link3
    global processed_data

    scene = Scene()

    # Set HDR skybox background
    loader = CubeTextureLoader()
    loader.setPath('https://threejs.org/examples/textures/cube/Bridge2/')  # HDR-like example set
    skybox = loader.load([
        'posx.jpg', 'negx.jpg',
        'posy.jpg', 'negy.jpg',
        'posz.jpg', 'negz.jpg'
    ])
    scene.background = skybox
    scene.environment = skybox

    # Camera setup
    camera = PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000)
    camera.position.set(10, 10, 10)  # Set a better view for the sphere
    camera.lookAt(0, 0, 0)  # Make sure the camera is pointing at the origin

    # Lights setup
    scene.add(AmbientLight('white', 0.6))
    directional = DirectionalLight('white', 1)
    directional.position.set(5, 10, 7)
    scene.add(directional)

    # # Create a sphere (representing a joint)
    # sphere_geometry = SphereGeometry(5, 32, 32)  # Radius of 5 with 32 segments for a smooth sphere
    # sphere_material = MeshStandardMaterial.new({'color': 0x00ffff})  # Corrected instantiation with new
    # sphere = Mesh(sphere_geometry, sphere_material)
    # Base = np.array([0, 0, 0])
    #     J2_Pos = np.array([T01[0, 3], T01[1, 3], T01[2, 3]])  # Joint 2
    #     J3_Pos = np.array([T02[0, 3], T02[1, 3], T02[2, 3]])  # Joint 3
    #     # Joint 4. it is assumed that the origin of joint 4 and 5 coincide.
    #     J4_Pos = np.array([T03[0, 3], T03[1, 3], T03[2, 3]])
    #     J5_Pos = np.array([T04[0, 3], T04[1, 3], T04[2, 3]])
    #     EE_Pos = np.array([TEE[0, 3], TEE[1, 3], TEE[2, 3]])
    #     # --- Horarm joints
    #     A_Pos = np.array([T0A[0, 3], T0A[1, 3], T0A[2, 3]])
    #     B_Pos = np.array([T0B[0, 3], T0B[1, 3], T0B[2, 3]])

    #     # Coordinates for each link.
    #     Link1 = list(zip(Base, J2_Pos))   # From base to Joint 2
    #     Link2 = list(zip(J2_Pos, J3_Pos))  # From Joint 2 to Joint 3
    #     Link3 = list(zip(J3_Pos, J4_Pos))  # From Joint 3 to Joint 4
    #     Link4 = list(zip(J4_Pos, J5_Pos))  # From Joint 4 to Joint 5
    #     Link5 = list(zip(J5_Pos, EE_Pos))  # From Joint 5 to the end effector
    #     # --- Horarm links
    #     LinkA = list(zip(J2_Pos, A_Pos))  # From Joint 2 to Joint A
    #     LinkB = list(zip(A_Pos, B_Pos))  # From Joint A to Joint B
    #     LinkC = list(zip(B_Pos, J4_Pos))  # From Joint B to Joint 4

    # # Position the sphere
    # sphere.position.set(0, 0, 0)  # Position at origin
	# # remember            x  z  y
    joint1 = create_joint(0, 0, 0)  # Create a joint at the origin
    joint2 = create_joint(1, 7, 1)  # Create a joint above the first one
    joint3 = create_joint(5, 5, 0)  # Create a joint to the right of the first one
    joint4 = create_joint(7, 5, 0)  # Create a joint at the origin

    link1 = create_link(joint1, joint2)  # Create a link between the first two joints
    link2 = create_link(joint2, joint3)  # Create a link between the first and third joints
    link3 = create_link(joint3, joint4)  # Create a link between the first and fourth joints

    # Add items to the scene
    scene.add(joint1)
    scene.add(joint2)
    scene.add(joint3)
    scene.add(joint4)
    scene.add(link1)
    scene.add(link2)
    scene.add(link3)




































    # Floor setup
    floor_geom = THREE.PlaneGeometry.new(100, 100)  # Corrected instantiation
    floor_material = THREE.MeshStandardMaterial.new({'color': 0x333333, 'metalness': 0.3, 'roughness': 0.8})
    floor = THREE.Mesh.new(floor_geom, floor_material)
    floor.rotation.x = -window.Math.PI / 2
    floor.position.y = -2
    scene.add(floor)

    # Renderer setup
    renderer = WebGLRenderer({'antialias': True})
    renderer.setSize(window.innerWidth, window.innerHeight)
    renderer.setPixelRatio(window.devicePixelRatio)
    document.body <= renderer.domElement

    # Controls setup
    OrbitControls(camera, renderer.domElement)

    # Resize handler
    def on_resize(event):
        camera.aspect = window.innerWidth / window.innerHeight
        camera.updateProjectionMatrix()
        renderer.setSize(window.innerWidth, window.innerHeight)

    window.addEventListener("resize", on_resize)

    animate()

def animate(*args):
    window.requestAnimationFrame(animate)
    
    # Optional: Animate the sphere (move it around or scale it)
    # angle = window.Math.sin(window.performance.now() / 1000) * window.Math.PI / 4
    # sphere.position.set(3 * window.Math.cos(angle), 2, 0)

    renderer.render(scene, camera)

init()