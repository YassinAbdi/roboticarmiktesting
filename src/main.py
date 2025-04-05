from browser import document, window

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
    # Calculate the distance and direction between the two joints
    dx = end_joint.position.x - start_joint.position.x
    dy = end_joint.position.y - start_joint.position.y
    dz = end_joint.position.z - start_joint.position.z
    length = window.Math.sqrt(dx * dx + dy * dy + dz * dz)

    # Create a cylinder geometry for the link
    cylinder_geometry = THREE.CylinderGeometry.new(1, 1, length, 32)  # Radius of 1 and 32 segments for smoothness

    # Create a mesh for the link
    cylinder_material = MeshStandardMaterial.new({'color': 0x00ff00})  # Green color for the link
    cylinder = Mesh(cylinder_geometry, cylinder_material)

    # Position the cylinder to be in the middle of the two joints
    cylinder.position.set(
        (start_joint.position.x + end_joint.position.x) / 2,
        (start_joint.position.y + end_joint.position.y) / 2,
        (start_joint.position.z + end_joint.position.z) / 2,
    )
    
    # Calculate the direction vector and normalize it
    direction = window.THREE.Vector3.new(dx, dy, dz).normalize()

    # Calculate the axis of rotation: we assume the "up" vector as [0, 1, 0]
    up = window.THREE.Vector3.new(0, 1, 0)

    # Calculate the rotation axis using the cross product
    rotation_axis = up.cross(direction)

    # Calculate the angle between the "up" vector and the direction vector
    angle = window.Math.acos(up.dot(direction))

    # Create a quaternion for the rotation
    quaternion = window.THREE.Quaternion.new()
    quaternion.setFromAxisAngle(rotation_axis, angle)

    # Apply the quaternion rotation to the cylinder
    cylinder.rotation.setFromQuaternion(quaternion)

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

    # # Position the sphere
    # sphere.position.set(0, 0, 0)  # Position at origin
    joint1 = create_joint(0, 0, 0)  # Create a joint at the origin
    joint2 = create_joint(0, 5, 0)  # Create a joint above the first one
    joint3 = create_joint(5, 5, 0)  # Create a joint to the right of the first one

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
