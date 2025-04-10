from browser import document, window
from  pyweb3d import *
from javascript import UNDEFINED as undefined
        
GCodeLoader = window.THREE.GCodeLoader.new
OrbitControls = window.THREE.OrbitControls.new
RGBELoader = window.THREE.RGBELoader.new

#Variables for setup

camera = None
renderer = None
scene = None
banchy = None

def init():
	global camera
	global renderer
	global scene
	global banchy

	#Create scene
	scene = Scene()
	scene.background = Color( 0x333333 )
	scene.environment = RGBELoader().load( 'textures/venice_sunset_1k.hdr' )
	scene.environment.mapping = EquirectangularReflectionMapping

	grid = GridHelper( 20, 40, 0xffffff, 0xffffff )
	grid.material.opacity = 0.2
	grid.material.depthWrite = False
	grid.material.transparent = True
	scene.add( grid )

	fov = 45
	aspect = window.innerWidth / window.innerHeight
	near = 1
	far = 150

	#Camera setup
	camera = PerspectiveCamera(fov, aspect, near, far)
	camera.position.x = 8
	camera.position.y = 1
	camera.position.z = 10

	ambientLight = AmbientLight( 'white', 2 )
	scene.add( ambientLight )

	DirectLight = DirectionalLight( 'white', 3.5 )
	DirectLight.position.set( 100, 100, 100 )
	scene.add( DirectLight )

	hemiLight = HemisphereLight('white', 'darkslategrey', 5)
	scene.add(hemiLight)

	#Renderer
	renderer = WebGLRenderer( { 'antialias': True } )
	renderer.setPixelRatio( window.devicePixelRatio )
	renderer.setSize( window.innerWidth, window.innerHeight )
	document.body.appendChild( renderer.domElement )

	controls = OrbitControls(camera, renderer.domElement)

	#Load Model
	loader = GCodeLoader()

	def loadGcode(gcode):
		gcode.scale.set( 0.1, 0.1, 0.1 )
		gcode.position.set( -10, 0, 10 )
		scene.add( gcode )
		animate(0)

	def onError(error):
		print(error)

	loader.load( 'models/gcode/benchy.gcode', loadGcode, undefined, onError)

def onWindowResize(resize):

	camera.aspect = window.innerWidth / window.innerHeight
	camera.updateProjectionMatrix()

	renderer.setSize( window.innerWidth, window.innerHeight )

window.addEventListener( 'resize', onWindowResize )

def animate(time):
	window.requestAnimationFrame( animate )
	renderer.render( scene, camera )
	
init()
