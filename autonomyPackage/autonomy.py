from threading import Event, Thread
import queue
from multiprocessing import Process, Pipe
from dronekit import connect, VehicleMode


coordsPipe, coordsReceiver = Pipe()  # sender, receiver
gesturesPipe, gesturesReceiver = Pipe()
n = 30 # size of the grid
mode = "START"  # TODO: globals arent a good practice, look into inter-thread communication

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    MAV_FRAME_BODY_OFFSET_NED - makes velocity component relative to drone's current heading (Currently used)
                                ex. North = front of drone, South = back of drone, East = right, West = left
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
        
# stub functions for coords/gestures
def coordinatesMain(sendPipe, n):
    pass

def gesturesMain(sendPipe):
    pass

class CommunicationThread(threading.Thread):
    """
    Handles incoming communcation from coordinates and gestures
    """
    def __init__(self, sendQueue, recvPipe, stopRequest):
        super(communicationThread, self).__init__()
        self.sendQueue = sendQueue  # pass data received to main thread
        self.stopRequest = stopRequest
        self.recvPipe = recvPipe

    def run(self):
        while not self.stoprequest.isSet():            
            res = recvPipe.recv()
            sendQueue.put(res)

    def join(self):
        super(CommunicationThread, self).join()

        
class AutonomyController:
    def __init__(self):

        
        self.currentlyFlying = False  # whether the drone is currently executing a flight command
        self.command = None
        self.currentCoordinate = [0, 0, 0]        
        self.error = False # error code that indicates a need to land, implement an interrupt?

        self.coordsPipe = Pipe()
        self.gesturesPipe = Pipe()
        self.coordinatesProcess = Process(target = coordinatesMain, args = (coordsPipe, n))        
        self.gesturesProcess = Process(target = gesturesMain, args = (gesturesPipe,))

        coordsDataQueue = queue.Queue()
        gestureDataQueue = queue.Queue()
        self.stopRequest = threading.Event() # signal threads to stop
        self.commThreadCoords = CommunicationThread(coordsDataQueue, coordsPipe, stopRequest)
        self.commThreadGesture = CommunicationThread(gestureDataQueue, gesturesPipe, stopRequest) 

        # check if we need to lock this if one thread is reading and the other is writing
        self.coordLock = Lock()  


    def main(self):
        # start the threads, then do cleanup before shutdown once flightCtrl terminates
        self.commThreadCoords.start()
        self.commThreadGesture.start()
        self.coordinatesProcess.start()
        self.gesturesProcess.start()

        #self.flightCtrl()
        mode = "LAND"
        
        
        self.commThreadCoords.join()
        self.commThreadGesture.join()
        print("Threads joined")
        self.coordinatesProcess.join()
        self.gesturesProcess.join()


    def takeoff(self, altitude):

        self.vehicle = connect(connection_string, wait_ready=True)
        while self.vehicle.is_armable() == True:
           self.vehicle.mode = dronekit.VehicleMode("GUIDED")
           self.vehicle.armed == True
           self.vehicle.simple_takeoff(altitude)
        
            

    def move(self, move_vector):
        pass

    def land(self):

        self.stopRequest.set()
        #check location
        self.vehicle.mode = VehicleMode("LAND")
        #when altitude reaches 0, send message 'landed'
        self.vehicle.close()
        stil.stop()

    def flight_control(self):
        # The main decision process for autonomy.
        # This function should terminate when the process is ready to terminate.

        while(self.mode != "LAND"):

            # Check if there are any new instructions. 
            # If so, get the latest one (any missed instruction from past are discarded)
            if self.gestureDataQueue.empty():
                continue
            else:
                while not gestureDataQueue.empty()
                self.command = self.gestureDataQueue.get(block=false)
         

            if self.command == 'land':  # a land instruction will need to be determined
                self.land()
                break
            else:
                # get current drone location.   Delete any old location updates
                while not coordsDataQueue.empty():
                    self.currentCoordinate.get(block=false)

                valid = check_location(self.currentCoordinate[0], self.currentCoordinate[1])

                if (valid):
                    self.fly()                    
                    
                else:
                    ## TODO: handle invalid location
                    continue
        

    def check_location(self, primary, secondary) -> bool:
        pass
        #calculate location and check if it is safe to move to that location

        
    """
    x > 0 => fly North
    x < 0 => fly South
    y > 0 => fly East
    y < 0 => fly West
    z < 0 => ascend
    z > 0 => descend
    """
    def fly(self, x,y,z, FLIGHTDURATION=5):
        currentlyFlying = True
        if self.check_location:
            send_ned_velocity(x, y, z, FLIGHTDURATION)
            send_ned_velocity(0, 0, 0, 1)
        else:
            print("Invalid movement\n")
        currentlyFlying = False
