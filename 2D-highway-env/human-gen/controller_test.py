import pygame

# Define some colors.
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

# This is a simple class that will help us print to the screen.
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


pygame.init()

# Set the width and height of the screen (width, height).
screen = pygame.display.set_mode((500, 200))
pygame.display.set_caption("AI fly")
# Loop until the user clicks the close button.
done = False
# Used to manage how fast the screen updates.
clock = pygame.time.Clock()
# Initialize the joysticks.
pygame.joystick.init()
# Get ready to print.
textPrint = TextPrint()

# -------- Main Program Loop -----------
while not done:

    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

    screen.fill(WHITE)
    textPrint.reset()
    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        textPrint.tprint(screen, "Joystick {}".format(i))
        textPrint.indent()

        # for i in range(axes):
            # axis = joystick.get_axis(i)
            # textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(i, axis))
        # textPrint.unindent()

        axis0 = joystick.get_axis(0)

        textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(0, axis0))

        axis1 = joystick.get_axis(1)
        textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(1, axis1))

        axis2 = joystick.get_axis(2)
        textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(2, axis2))

        axis3 = joystick.get_axis(3)
        textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(3, axis3))

    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    #
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
    # Limit to 20 frames per second.
    clock.tick(20)
pygame.quit()