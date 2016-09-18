import pygame  
import socket, time

host = "192.168.0.111"
port = 8000
sock_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_send.connect((host, port))

pygame.init()

#Loop until the user clicks the close button.
done = False

# Initialize the joysticks
pygame.joystick.init()

# For X axis
OldMax1 = 1.0
OldMin1 = -1.0
NewMax1 = 4000
NewMin1 = 2000
OldRange1 = (OldMax1 - OldMin1)  
NewRange1 = (NewMax1 - NewMin1)  

# For Y axis
OldMax2 = -1.0
OldMin2 = 1.0
NewMax2 = 4000
NewMin2 = 2000
OldRange2 = (OldMax2 - OldMin2)  
NewRange2 = (NewMax2 - NewMin2)

xbox_joy_index = 0
joystick_name = "xbox"

joystick_count = pygame.joystick.get_count()
print "Joystick Count: ", joystick_count

joystick = None
for i in range(joystick_count):
    joystick = pygame.joystick.Joystick(i)
    joystick.init()

    # Get the name from the OS for the controller/joystick
    name = joystick.get_name().lower()
    if(joystick_name in name):
        print "Joystick Found... ", name
        xbox_joy_index = i

pitch_axis = 3
roll_axis = 4
throttle_axis = 1
yaw_axis = 0

# -------- Main Program Loop -----------
while done==False:       
    # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
    for event in pygame.event.get(): 
        if event.type == pygame.JOYBUTTONDOWN:
            print "Joystick button pressed."
        if event.type == pygame.JOYBUTTONUP:
            print"Joystick button released."
            
    OldValue2 = joystick.get_axis(pitch_axis)
    scaled_pitch = int((((OldValue2 - OldMin2) * NewRange2) / OldRange2) + NewMin2)
    
    OldValue1 = joystick.get_axis(roll_axis)
    scaled_roll = int((((OldValue1 - OldMin1) * NewRange1) / OldRange1) + NewMin1)

    OldValue1 = joystick.get_axis(yaw_axis)
    scaled_yaw = int((((OldValue1 - OldMin1) * NewRange1) / OldRange1) + NewMin1)

    OldValue2 = joystick.get_axis(throttle_axis)
    scaled_throttle = int((((OldValue2 - OldMin2) * NewRange2) / OldRange2) + NewMin2)

    print "Pitch: ", scaled_pitch, "Roll: ", scaled_roll, "Yaw: ", scaled_yaw, "Throttle: ", scaled_throttle

    buttons = joystick.get_numbuttons()
    button_data = ""
    for i in range( buttons ):
        button = joystick.get_button( i )
        button_data += str(button)
   
    # Hat switch. All or nothing for direction, not like joysticks.
    # Value comes back in an array.
    hats = joystick.get_numhats()
    hat_data = ""
    for i in range( hats ):
        hat = joystick.get_hat( i )
        button_data += str(hat)
        
    sock_data = "P=" + str(scaled_pitch) + "&R=" + str(scaled_roll) + "&Y=" + str(scaled_yaw) + "&T=" + str(scaled_throttle) + "&B=" + str(button_data) + "\n"
    sock_send.send(sock_data)
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit ()
