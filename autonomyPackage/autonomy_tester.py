from autonomy import *

if __name__ == '__main__':
	print("Creating autonomy controller")
	controller = AutonomyController()
	coordsPipe.send("hello")
	gesturesPipe.send("hello")
	print("Run main")
	controller.main()
