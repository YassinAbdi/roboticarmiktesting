from browser import document, window

THREE = window.THREE

# Shortcuts to JS constructors
PerspectiveCamera = THREE.PerspectiveCamera.new
Scene = THREE.Scene.new
WebGLRenderer = THREE.WebGLRenderer.new
PlaneGeometry = THREE.PlaneGeometry.new
PointsMaterial = THREE.PointsMaterial
Points = THREE.Points.new
BufferGeometry = THREE.BufferGeometry.new
Float32Array = window.Float32Array.new
LineBasicMaterial = THREE.LineBasicMaterial
Line = THREE.Line.new
MeshStandardMaterial = THREE.MeshStandardMaterial.new
Mesh = THREE.Mesh.new
AmbientLight = THREE.AmbientLight.new
DirectionalLight = THREE.DirectionalLight.new
Color = THREE.Color.new
OrbitControls = window.THREE.OrbitControls.new
CubeTextureLoader = THREE.CubeTextureLoader.new

# Globals
camera = None
renderer = None
scene = None
joints = []
links = []

def init():
    global camera, renderer, scene, joints, links

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

    # Camera
    camera = PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000)
    camera.position.set(5, 5, 5)

    # Lights
    scene.add(AmbientLight('white', 0.6))
    directional = DirectionalLight('white', 1)
    directional.position.set(5, 10, 7)
    scene.add(directional)

    # Create 3 points (joints) for the robotic arm
    joint_material = PointsMaterial.new({ 'color': 0x00ffff, 'size': 1 , 'sizeAttenuation': False })
    joint_geom = BufferGeometry()
    joints = [
        Points(joint_geom, joint_material),  # Base
        Points(joint_geom, joint_material),  # Elbow
        Points(joint_geom, joint_material)   # Wrist
    ]
    
    # Positioning the points (joints)
    joints[0].position.set(0, 0, 0)  # Base
    joints[1].position.set(3, 2, 0)  # Elbow
    joints[2].position.set(6, 1, 0)  # Wrist
    
    for joint in joints:
        scene.add(joint)
        print(joint.position)  # Debugging: print joint positions

    # Create linkages (lines) between the points
    line_material = LineBasicMaterial.new({ 'color': 0xff0000 })
    line_geom = BufferGeometry()
    
    # Link base to elbow
    line_geom.setAttribute("position", THREE.Float32BufferAttribute.new(Float32Array([0, 0, 0, 3, 0, 0]), 3))
    link1 = Line(line_geom, line_material)
    links.append(link1)
    scene.add(link1)

    # Link elbow to wrist
    line_geom.setAttribute("position", THREE.Float32BufferAttribute.new(Float32Array([0, 0, 0, .1, 0, 0]), 3))
    link2 = Line(line_geom, line_material)
    links.append(link2)
    scene.add(link2)

    # Floor
    floor_geom = PlaneGeometry(100, 100)
    floor_material = MeshStandardMaterial({ 'color': 0x333333, 'metalness': 0.3, 'roughness': 0.8 })
    floor = Mesh(floor_geom, floor_material)
    floor.rotation.x = -window.Math.PI / 2
    floor.position.y = -2
    scene.add(floor)

    # Renderer
    renderer = WebGLRenderer({ 'antialias': True })
    renderer.setSize(window.innerWidth, window.innerHeight)
    renderer.setPixelRatio(window.devicePixelRatio)
    document.body <= renderer.domElement

    # Controls
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

    # Rotate the robotic arm joints
    angle = window.Math.sin(window.performance.now() / 1000) * window.Math.PI / 4
    joints[1].position.set(3, 2, 0)
    joints[2].position.set(6 * window.Math.cos(angle), 0, 6 * window.Math.sin(angle))

    # Update linkages based on joint positions
    links[0].geometry.setAttribute("position", THREE.Float32BufferAttribute.new(Float32Array([0, 1, 0, 3, 0, 0]), 3))
    links[1].geometry.setAttribute("position", THREE.Float32BufferAttribute.new(Float32Array([3, 0, 0, 6 * window.Math.cos(angle), 0, 6 * window.Math.sin(angle)]), 3))

    renderer.render(scene, camera)

init()
