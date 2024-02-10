from engi1020.arduino.api import *
import pygame
import threading
import time
import cv2
from numpy import ndarray
import level_5

WIDTH, HEIGHT = 1070, 600
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Arduino Mayhem!")
pygame.font.init()

SPEECH_SIZE = round((40/600)*HEIGHT)
SPEECH_FONT = pygame.font.SysFont('timesnewroman', SPEECH_SIZE)
SPEECH_DELAY = 0.03

FIRST_LINES = {"Other than making codes, my second favourite pass-time is painting.":'neutral',
              "My preferred colours are red, green, blue!":"happy",
              "You can do a lot with red, green and blue!":"happy",
              "I doubt you're artistic enough to know how that even works":"angry",
              "I know for a fact you cheated on the previous puzzle, you and your pesky arduino":"angry"}

SECOND_LINES = {"For this puzzle, I'm going to look you right in the eye!":"angry",}

THIRD_LINES = {"I'll be able to see you!":"neutral",
                "Or maybe not, I don't have a face recognition algorithm.":"surprised",
                "You might want to move your popup windows to see everything (just the top of this window and camera feed)":"neutral",
               "Also, I'll take me a few seconds to open my eyes":"neutral",
                "Anyhow, show me the colour GREEN":"happy"}

LAST_LINES = {".....           ":"terrified",
               "I'm sure you've noticed that you can move around in the window...":"angry",
               "You should know that I have some friends that can stop you from doing that...":"angry",
               "I never though I'd resort to violence":"surprised",
               "LEVEL 5!!!!":"neutral"}



current_text = ''

PLAYER_WIDTH, PLAYER_HEIGHT = round((25/854)*WIDTH), round((25/480)*HEIGHT)
LIGHTBULB_WIDTH, LIGHTBULB_HEIGHT = round((100/854)*WIDTH), round((120/480)*HEIGHT)

LIGHTBULB_MAN_SPRITES = {"neutral":"lightbulb_man_neutral.png",
                         "happy":"lightbulb_man_happy.png",
                         "surprised":"lightbulb_man_surprised.png",
                         "terrified":"lightbulb_man_terrified.png",
                         "angry":"lightbulb_man_angry.png",
                         "transparent":"lightbulb_man_transparent.png"}
CURRENT_LIGHTBULB_MAN_STATE = None

solid_objects = []
event_objects = []
event_rectangles = []
mouse_event_data = (False, 0) #The first value corresponds to mouse being pressed and the second is mouse pos
text_typed = ''
current_frame = None
camera_puzzle_solved = False
progress_towards_camera_puzzle = 0
camera_active = False
CAMERA_WINDOW_WIDTH = 240
CAMERA_WINDOW_LENGTH = 320
DESIRED_RADIUS = round((3/8)*CAMERA_WINDOW_WIDTH)


WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)

running = True
player_roam = True
FPS = 60

VEL = round((8/600)*WIDTH)

def set_lightbulbman_state(state):
    """Takes a string as an input corresponding to a key in the dictionary called
    LIGHTBULB_MAN SPRITES. Sets the variable CURRENT_LIGHTBULB_MAN_STATE to the
    corresponding asset to the passed in emotion"""
    global CURRENT_LIGHTBULB_MAN_STATE

    if state not in LIGHTBULB_MAN_SPRITES.keys():
        print(f"Cannot find state {state}")
    else:
        lightbulb_man_image = pygame.image.load(f"Assets\{LIGHTBULB_MAN_SPRITES[state]}")
        CURRENT_LIGHTBULB_MAN_STATE = pygame.transform.scale(lightbulb_man_image,
                                                             (LIGHTBULB_WIDTH, LIGHTBULB_HEIGHT))

def will_collide(object, direction, object_vel):
    """Determines whether the object passed in will or will not collide, with the given velocity
             with any object in the solid_objects list"""
    if direction == "sideways":
        theoretical_object = pygame.Rect(object.x + object_vel, object.y, object.width, object.height)
    elif direction == "up_down":
        theoretical_object = pygame.Rect(object.x, object.y+ object_vel, object.width, object.height)
    for solid_object in solid_objects:
        if solid_object.colliderect(theoretical_object):
            return True
    return False


def draw_window(player, lightbulb_man):
    """Takes in the player and lightbulb_man rectangles. Renders every object, including
                objects defined in global iterables such as event_objects and event_rectangles.
                This allows event threads to also use this function."""
    WIN.fill(BLACK)

    pygame.draw.rect(WIN, RED, player)
    WIN.blit(CURRENT_LIGHTBULB_MAN_STATE, (lightbulb_man.x, lightbulb_man.y))

    speech = SPEECH_FONT.render(
        str(current_text), 1, WHITE)
    speech = pygame.transform.scale(speech, (round(speech.get_size()[0]*(2/3)), round(speech.get_size()[1]*(2/3))))
    if speech.get_size()[0] > WIDTH - LIGHTBULB_WIDTH:
        ratio = (WIDTH-LIGHTBULB_WIDTH)/speech.get_size()[0]
        scaled_width = round(speech.get_size()[0] * ratio)
        scaled_height = round(speech.get_size()[1] * ratio)
        speech = pygame.transform.scale(speech, (scaled_width, scaled_height))

    WIN.blit(speech, (10, 10))

    for object in event_objects:
        WIN.blit(object[1], (object[0].x, object[0].y))

    for object in event_rectangles:
        pygame.draw.rect(WIN, object[1], object[0])


    pygame.display.update()

def player_move(keys_pressed, player, lightbulb_man):
    """Takes in the keys_pressed and the player. Used to give player movement while respecting
                boundaries, and aided by the will_collide function."""
    if (keys_pressed[pygame.K_LEFT] or keys_pressed[pygame.K_a]) and player.x - VEL > 0 and not will_collide(player, "sideways", -VEL):  # LEFT
        player.x -= VEL
    if (keys_pressed[pygame.K_RIGHT] or keys_pressed[pygame.K_d]) and (player.x + VEL + PLAYER_WIDTH < WIDTH) and not will_collide(player, "sideways", VEL):  # RIGHT
        player.x += VEL
    if (keys_pressed[pygame.K_UP] or keys_pressed[pygame.K_w]) and player.y - VEL > SPEECH_SIZE and not will_collide(player, "up_down", -VEL):  # UP
        player.y -= VEL
    if (keys_pressed[pygame.K_DOWN] or keys_pressed[pygame.K_s]) and player.y + VEL + PLAYER_HEIGHT < HEIGHT and not will_collide(player, "up_down", VEL):  # DOWN
        player.y += VEL

def print_lines(lines, auto_skip_to_event = True, wait_to_skip=False):
    '''If wait_to_skip is true, then there will be a delay of 2 seconds after the last line.
         If auto_skip_to_event is true, then it will skip without user input after last line.'''
    global current_text
    current_text = ''

    line_index = 0
    skip_to_event = False

    lines_dict = lines
    lines = list(lines.keys())

    for line in lines:
        set_lightbulbman_state(lines_dict[line])
        while True:
            if line_index <= len(line):
                current_text = line[:line_index]
                line_index += 1
                time.sleep(SPEECH_DELAY)
            else:
                if lines.index(line) == len(lines)-1:
                    skip_to_event = True
                line_index = 0
                time.sleep(SPEECH_DELAY)
                break
        if skip_to_event:
            if not auto_skip_to_event:
                check_key_pressed()
            elif wait_to_skip:
                time.sleep(2)
            else:
                break
        else:
            check_key_pressed()

def check_key_pressed():
    """Breaks out of loop when space is pressed"""
    while True:
        keys_pressed = pygame.key.get_pressed()
        if keys_pressed[pygame.K_SPACE] == True:
            break


def camera_image_processor():
    """This function is meant to be threaded. While the level is running, the loop
    in this function will update the global variable current frame with the current
    frame"""
    global current_frame, camera_active
    capture = cv2.VideoCapture(0)
    while running:
        ret, frame = capture.read()
        frame = cv2.resize(frame, (CAMERA_WINDOW_LENGTH,CAMERA_WINDOW_WIDTH))
        if ret:
            current_frame = frame
            camera_active = True

def filter_color(rgb_image, lower_bound, upper_bound):
    """Given an RGB image and two HSV tuples for lower and upper thresholding bounds,
    this function will return a black and white image where white represents the pixels
    that have met the threshold. This function uses the OpenCV functions cvtColour and inRange"""
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    return mask

def get_contours(mask):
    """Given a black and white image where white represents pixels that have met a threshold, this function
    uses OpenCV's findContours to locate the contours or 'edges' of the white areas and it returns these OpenCV
    contour objects in a list."""
    contours, hirearchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_contours(rgb_image,contours):
    """Given an RGB image on which to draw the contours and a list of contours, which are NumPy arrays,
    this function finds the contour with has the greatest area (cv2.contourArea(contour)) and, if this contour
    is at least 5 pixels in area, draws this contour onto the RGB image. Based on the radius of this largest contour,
    this function updates the variable progress_towards_camera_puzzle to that radius. If the radius of the largest contour
    reaches DESIRED_RADIUS, the global variable camera_puzzle_solved is set to True, which prompts a change in the level events."""
    global camera_puzzle_solved, progress_towards_camera_puzzle

    biggest_contour = ndarray(shape= (1,1,1))
    biggest_contour_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > biggest_contour_area:
            biggest_contour_area = area
            biggest_contour = contour
    if (len(contours) > 0) and (biggest_contour_area > 5): #make sure you have at least one decent contour
        ((x,y),radius) = cv2.minEnclosingCircle(biggest_contour)
        cx,cy = get_contour_center(biggest_contour)
        if radius >= DESIRED_RADIUS:
            camera_puzzle_solved = True
        progress_towards_camera_puzzle = radius
        cv2.circle(rgb_image, (cx,cy),int(radius),(0,0,255),1)

def get_contour_center(contour):
    """Given a contour, this function uses the OpenCV function "moments" and some math to
    find the center of area of that contour"""
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
        cx= round(M['m10']/M['m00'])
        cy= round(M['m01']/M['m00'])
    return cx, cy

def event_1():
    """This event uses the functions filter_colour, get_contours, draw_contours, and get_contour_center to
    give the player an experience where they have to show the lightbulb man a certain colour. In this function
    is a while True loop that is constantly filtering out the largest color within the colour bounds (either green or blue)
    and showing the minimum enclosing circle of that contour in the window "Lightbulb Man POV" When the global variable
    camera_puzzle_solved is set to True (which happens in the draw_contours function), this function will check a few things.

    1. If the camera puzzle is sloved in the first 500 iterations of the loop (very quickly) the function assumes that the player already
    had the colour green on their screen and goes to another loop iteration, resetting the thresholding bounds to blue.
    2. If the camera puzzle is solved again within iteration 500-1000 with the blue threshold, the function assumes that the player already
    had green and blue on their screen and returns.
    3. EVENT_LINES_1 and EVENT_LINES_2 are printed based on what happens in this event."""
    global current_frame, camera_puzzle_solved, player, solid_objects, event_rectangles, progress_towards_camera_puzzle
    EVENT_LINES_1 = {"Oh, you already have green... show me BLUE instead!":"surprised"}
    EVENT_LINES_2 = {"You had blue and green already covering half your screen!": "terrified"}

    #Bounds below defined in HSV
    GREEN_LOWER_BOUND = (40, 100, 100)
    GREEN_UPPER_BOUND = (85, 255, 255)
    BLUE_LOWER_BOUND = (100, 100, 100)
    BLUE_UPPER_BOUND = (140, 255, 255)

    player.x = 200
    player.y = 200

    progress_bar_background = pygame.Rect(0, 45, round(WIDTH / 2),
                                          round((1 / 10) * HEIGHT))
    event_rectangles.append([progress_bar_background, WHITE])  # Now the draw_windows will draw what we want
    solid_objects.append(progress_bar_background)

    progress_bar_rect = pygame.Rect(0, 45, round(WIDTH / 2),
                                    round((1 / 10) * HEIGHT))
    event_rectangles.append([progress_bar_rect, GREEN])

    bounds = (GREEN_LOWER_BOUND, GREEN_UPPER_BOUND)
    iteration = 0
    already_green = False
    while True:
        mask = filter_color(current_frame, bounds[0], bounds[1])
        contours = get_contours(mask)
        draw_contours(current_frame, contours)

        cv2.imshow("Lightbulb Man POV", current_frame)
        cv2.waitKey(1)

        event_rectangles.remove([progress_bar_rect, GREEN])
        progress_bar_rect = pygame.Rect(0, 45,
                                        round((WIDTH / 2) * (progress_towards_camera_puzzle / DESIRED_RADIUS)),
                                        round((1 / 10) * HEIGHT))
        event_rectangles.append([progress_bar_rect, GREEN])

        if camera_puzzle_solved and iteration in range(0, 500):
            bounds = (BLUE_LOWER_BOUND, BLUE_UPPER_BOUND)
            camera_puzzle_solved = False
            already_green = True
            iteration = 499
            event_rectangles.remove([progress_bar_rect, GREEN])
            progress_bar_rect = pygame.Rect(0, 45,
                                            1,
                                            round((1 / 10) * HEIGHT))
            event_rectangles.append([progress_bar_rect, GREEN])
            print_lines(EVENT_LINES_1)
        elif camera_puzzle_solved and iteration in range(500, 1000) and already_green == True:
            print_lines(EVENT_LINES_2, auto_skip_to_event=False)
            break
        elif camera_puzzle_solved:
            break

        iteration += 1

    cv2.destroyWindow("Lightbulb Man POV")
    event_rectangles = []
    solid_objects.remove(progress_bar_background)




def events():
    """This function is threaded and contains the events of the level in chronological order"""
    #Allow pygame to start
    time.sleep(5)
    global current_text, running, camera_puzzle_active

    oled_clear()
    print_lines(FIRST_LINES, wait_to_skip= True, auto_skip_to_event=False)

    oled_clear()
    oled_print("DID NOT")
    oled_print("CHEAT!")

    print_lines(SECOND_LINES, wait_to_skip=True, auto_skip_to_event=False)

    oled_clear()

    print_lines(THIRD_LINES, wait_to_skip=True)

    camera_thread = threading.Thread(target=camera_image_processor, daemon=False)
    camera_thread.start()
    ####### EVENT 1
    while True:
        if camera_active:
            break
    event_1()
    #######

    print_lines(LAST_LINES)

    time.sleep(3)
    running = False

def main():
    """Starts the thread and initializes objects, then handles the main game loop."""
    events_thread = threading.Thread(target=events, daemon=True)
    events_thread.start()

    global player, mouse_event_data, text_typed
    player = pygame.Rect(200, 200, PLAYER_WIDTH, PLAYER_HEIGHT)
    lightbulb_man = pygame.Rect(WIDTH-LIGHTBULB_WIDTH, 0, LIGHTBULB_WIDTH, LIGHTBULB_HEIGHT)
    solid_objects.append(lightbulb_man)
    set_lightbulbman_state('neutral')

    clock = pygame.time.Clock()

    while True: #Check if Arduino is connected
        try:
            digital_write(4, True)
            digital_write(4, False)
            break
        except:
            print("Arduino not working...")

    while running:

        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_event_data = (True, event.pos)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_BACKSPACE:
                    text_typed = text_typed[:-1]
                else:
                    text_typed += event.unicode

        keys_pressed = pygame.key.get_pressed()
        if player_roam:
            player_move(keys_pressed, player, lightbulb_man)
        draw_window(player, lightbulb_man)

    level_5.main()

if __name__ == "__main__":
    main()