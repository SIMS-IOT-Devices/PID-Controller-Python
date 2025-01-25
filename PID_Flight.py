import pygame
import math

# Initialize pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Flight Simulation with Physics")

# Colors
WHITE = (255, 255, 255)
BLUE = (135, 206, 235)
GREEN = (34, 139, 34)
BLACK = (0, 0, 0)

# Clock
clock = pygame.time.Clock()

# Constants
AIR_DENSITY = 1.225  # kg/m^3 (at sea level)
GRAVITY = 9.81  # m/s^2
TIME_STEP = 1 / 60  # 60 FPS

# Aircraft class
class Aircraft:
    def __init__(self, x, y, mass, wing_area):
        self.x = x
        self.y = y
        self.vx = 0  # Velocity in x-direction
        self.vy = 0  # Velocity in y-direction
        self.angle = 0  # Angle of the aircraft (degrees)
        self.mass = mass  # kg
        self.wing_area = wing_area  # m^2
        self.thrust = 7000  # Thrust force (N)
        # self.thrust = 0  # No thrust force (N)
        self.max_thrust = 10000  # Maximum thrust (N)
        self.coefficient_lift = 1.2  # Coefficient of lift
        self.coefficient_drag = 0.05  # Coefficient of drag
        # Initialize the PID controller
        # Kp: Proportional gain. Ki: Integral gain. Kd: Derivative gain.
        self.Kp = 1.0
        self.Ki = 0.5
        self.Kd = 0.1
        # Initialize variables
        self.prev_error = 0
        self.integral_PID = 0

    # Cacuate forces
    def calculate_forces(self):
        # Velocity magnitude
        velocity = math.sqrt(self.vx**2 + self.vy**2)

        # Lift
        lift = (
            0.5
            * AIR_DENSITY
            * velocity**2
            * self.coefficient_lift
            * self.wing_area
        )
         
        # Drag
        drag = (
            0.5
            * AIR_DENSITY
            * velocity**2
            * self.coefficient_drag
            * self.wing_area
        )

        # Gravity
        gravity = self.mass * GRAVITY
      
        return lift, drag, gravity

    # Update controller calculations and fiscal world constants and forces
    def update(self, keys):

        # START of user manua controls for thrust and pitch ---------------------------------------
        if keys[pygame.K_LEFT]:  # Pitch up
            self.angle += 0.1
        if keys[pygame.K_RIGHT]:  # Pitch down
            self.angle -= 0.1
        # END of user manua controls for thrust and pitch -----------------------------------------


        # # START of automatic controls for pitch without PID controller ----------------------------
        # error = self.vy
        # if error<0:
        #     self.angle += 0.2
        # if error>0:
        #     self.angle -= 0.2
        # # END of Automatic controls for pitch without PID controller ------------------------------


        # # START of a PID controller on Vy in order to bring Vy to zero for trim flight ------------
        # # Calculate error, any Vy is an error in trim fight
        # error = float(self.vy)
        # # Update integral (accumulation of error over time)
        # self.integral_PID = self.integral_PID + error * TIME_STEP
        # # Calculate derivative (rate of change of error)
        # derivative = (error - self.prev_error) / TIME_STEP
        # # PID output for Vy to converge to zero
        # output = self.Kp * error + self.Ki * self.integral_PID + self.Kd * derivative
        # # Approximation of the phisical relation between angle ALPHA and Vy
        # output_angle = output/10 
        # # Update previous error
        # self.prev_error = error
        # # Final angle ALPHA needed to converge Vy to zero
        # self.angle = -output_angle
        # # END of a PID controller on Vy in order to bring Vy to zero for trim flight --------------

        # Calculate forces
        lift, drag, gravity = self.calculate_forces()

        # Convert angle to radians
        radian_angle = math.radians(self.angle)

        # Decompose thrust into x and y components
        thrust_x = self.thrust * math.cos(radian_angle)
        thrust_y = self.thrust * math.sin(radian_angle)

        # Net forces
        fx = thrust_x - drag
        fy = thrust_y + lift - gravity
        # print(fy, thrust_y, lift, gravity) 

        # Acceleration
        ax = fx / self.mass
        ay = fy / self.mass

        # Update velocity
        self.vx += ax * TIME_STEP
        self.vy += ay * TIME_STEP
        
        # Update position
        self.x += self.vx * TIME_STEP
        self.y += -self.vy * TIME_STEP

        # Keep within screen boundaries
        self.x %= WIDTH
        self.y = max(0, min(HEIGHT, self.y))  # Prevent from falling below ground
        
        # The plane hits the flore
        if self.y > HEIGHT-50:
            pygame.time.wait(3000)
            exit() 

        # Alfa update
        self.coefficient_lift = 0.08*math.degrees(math.radians(self.angle))  # Coefficient of lift
        # print(self.angle, self.coefficient_lift)
        self.coefficient_drag = 0.01667*math.degrees(math.radians(abs(self.angle)))

    # Draw the aircraft and the textual data on the screen
    def draw(self, surface):
        # Draw a simple triangle as the aircraft
        radian_angle = math.radians(self.angle)
        nose = (self.x + math.cos(radian_angle) * 20, self.y - math.sin(radian_angle) * 20)
        left_wing = (self.x + math.cos(radian_angle + math.pi * 0.75) * 15, self.y - math.sin(radian_angle + math.pi * 0.75) * 15)
        right_wing = (self.x + math.cos(radian_angle - math.pi * 0.75) * 15, self.y - math.sin(radian_angle - math.pi * 0.75) * 15)
        pygame.draw.polygon(surface, BLACK, [nose, left_wing, right_wing])

        # Print the angle and veocity on the screen       
        font_size = 36 # Font setup
        font = pygame.font.Font(None, font_size)  # None uses the default Pygame font

        radian_angle_str = "Alfa [deg]: " + str(round(math.degrees(math.radians(self.angle)),2)) # Ange in degrees
        number_text = font.render(radian_angle_str, True, BLACK)  # Text, antialiasing, color
        radian_angle_str_position = (10, 10)  # Position on the screen
        screen.blit(number_text, radian_angle_str_position) # Draw the text

        veocity_str_x = "Veocity_x [m/s]: " + str(round((self.vx))) # Ange in degrees
        number_text = font.render(veocity_str_x, True, BLACK)  # Text, antialiasing, color
        veocity_str_position_x = (10, 35)  # Position on the screen
        screen.blit(number_text, veocity_str_position_x) # Draw the text

        veocity_str_y = "Veocity_y [m/s]: " + str(round((-self.vy))) # Ange in degrees
        number_text = font.render(veocity_str_y, True, BLACK)  # Text, antialiasing, color
        veocity_str_position_y = (10, 60)  # Position on the screen
        screen.blit(number_text, veocity_str_position_y) # Draw the text

        CL_str = "CL: " + str(round((self.coefficient_lift),2)) # Ange in degrees
        number_text = font.render(CL_str, True, BLACK)  # Text, antialiasing, color
        veocity_str_position_y = (10, 85)  # Position on the screen
        screen.blit(number_text, veocity_str_position_y) # Draw the text

        CD_str = "CD: " + str(round((self.coefficient_drag),2)) # Ange in degrees
        number_text = font.render(CD_str, True, BLACK)  # Text, antialiasing, color
        veocity_str_position_y = (10, 110)  # Position on the screen
        screen.blit(number_text, veocity_str_position_y) # Draw the text

        T_str = "T: " + str(round((self.thrust))) # Ange in degrees
        number_text = font.render(T_str, True, BLACK)  # Text, antialiasing, color
        veocity_str_position_y = (10, 135)  # Position on the screen
        screen.blit(number_text, veocity_str_position_y) # Draw the text

        # Calculate forces
        lift, drag, gravity = self.calculate_forces()

        L_str = "L: " + str(round((lift))) # Ange in degrees
        number_text = font.render(L_str, True, BLACK)  # Text, antialiasing, color
        veocity_str_position_y = (10, 160)  # Position on the screen
        screen.blit(number_text, veocity_str_position_y) # Draw the text

        D_str = "D: " + str(round((drag))) # Ange in degrees
        number_text = font.render(D_str, True, BLACK)  # Text, antialiasing, color
        veocity_str_position_y = (10, 185)  # Position on the screen
        screen.blit(number_text, veocity_str_position_y) # Draw the text

        G_str = "G: " + str(round((gravity))) # Ange in degrees
        number_text = font.render(G_str, True, BLACK)  # Text, antialiasing, color
        veocity_str_position_y = (10, 210)  # Position on the screen
        screen.blit(number_text, veocity_str_position_y) # Draw the text

# Create an aircraft in the center of the screen
aircraft = Aircraft(WIDTH // 2, HEIGHT // 2, mass=500, wing_area=20)

# Main loop
running = True
while running:
    screen.fill(BLUE)  # Sky
    pygame.draw.rect(screen, GREEN, (0, HEIGHT - 50, WIDTH, 50))  # Ground

    # Enable keyboard input
    keys = pygame.key.get_pressed()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update the aircraft
    aircraft.update(keys)

    # Draw the aircraft
    aircraft.draw(screen)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
