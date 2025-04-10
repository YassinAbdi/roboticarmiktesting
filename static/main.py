from browser import document, window, ajax


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


def receive_data():
    # Request data from Flask
    def on_complete(req):
        if req.status == 200:
            data = window.JSON.parse(req.responseText)
            print("Received data:", data)
            
            # Handle "NaN" string and convert it back to actual NaN
            for array in data:
                # Convert "NaN" strings back to JavaScript NaN
                array = [window.NaN if x == "NaN" else x for x in array]
                print(f"Array: {array}")

        else:
            print("Error:", req.status)

    req = ajax.Ajax()
    req.bind('complete', on_complete)
    req.open('GET', '/send_data', True)
    req.send()

# Call the function to get the data
receive_data()

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
    joint4 = create_joint(0, 0, 0)  # Create a joint at the origin

    link1 = create_link(joint1, joint2)  # Create a link between the first two joints
    link2 = create_link(joint2, joint3)  # Create a link between the first and third joints

    # Add items to the scene
    scene.add(joint1)
    scene.add(joint2)
    scene.add(joint3)
    scene.add(link1)
    scene.add(link2)




































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