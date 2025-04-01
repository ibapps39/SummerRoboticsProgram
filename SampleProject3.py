"""
Introduction to micro:bit - LEDs, Buzzers, and Buttons
Author: Ian Brown
Date: April 2025

This program demonstrates fundamental micro:bit concepts for beginning robotics students:
- Using the built-in LED display for visual feedback
- Reading button inputs for interactivity
- Playing tones on a connected buzzer/speaker
- Creating simple interactive patterns

Compatible with BBC micro:bit V1 and V2
Recommended for Grades 6-8 introduction to physical computing
"""

from microbit import *
import music
import random

# Configuration
BUZZER_PIN = pin0  # Connect buzzer to pin 0
LED_BRIGHTNESS = 9  # Maximum brightness (0-9)

# ------------------ HELPER FUNCTIONS ------------------

def show_happy_face():
    """Display a happy face on the LED matrix"""
    display.show(Image.HAPPY)
    
def show_sad_face():
    """Display a sad face on the LED matrix"""
    display.show(Image.SAD)

def play_success_tone():
    """Play a short success melody on the buzzer"""
    music.play(music.POWER_UP, pin=BUZZER_PIN)
    
def play_error_tone():
    """Play an error tone on the buzzer"""
    music.play(music.WAWAWAWAA, pin=BUZZER_PIN)
    
def flash_border():
    """Flash the border LEDs of the display"""
    # Create a custom LED pattern for the border
    border = Image("99999:"
                   "90009:"
                   "90009:"
                   "90009:"
                   "99999")
    
    # Flash the border 3 times
    for _ in range(3):
        display.show(border)
        sleep(200)
        display.clear()
        sleep(200)

def led_chase():
    """Create an LED chase pattern around the display"""
    # Define the positions for the chase (clockwise around the border)
    chase_positions = [
        (0,0), (1,0), (2,0), (3,0), (4,0),  # Top row
        (4,1), (4,2), (4,3), (4,4),         # Right column
        (3,4), (2,4), (1,4), (0,4),         # Bottom row
        (0,3), (0,2), (0,1)                 # Left column
    ]
    
    # Light up each LED in sequence
    for x, y in chase_positions:
        display.clear()
        display.set_pixel(x, y, LED_BRIGHTNESS)
        sleep(100)

def random_sparkle(duration_ms=2000):
    """Create a random sparkling effect on the display"""
    end_time = running_time() + duration_ms
    
    while running_time() < end_time:
        x = random.randint(0, 4)
        y = random.randint(0, 4)
        brightness = random.randint(4, 9)
        display.set_pixel(x, y, brightness)
        sleep(50)
        display.set_pixel(x, y, 0)  # Turn off the pixel

def play_scale():
    """Play an ascending musical scale on the buzzer"""
    # C major scale notes
    notes = ['C4:2', 'D4:2', 'E4:2', 'F4:2', 'G4:2', 'A4:2', 'B4:2', 'C5:4']
    music.play(notes, pin=BUZZER_PIN)

# ------------------ INTERACTIVE DEMOS ------------------

def button_demo():
    """
    Demonstrate button inputs with visual and audio feedback
    
    Instructions:
    - Press button A to show a happy face and play a success tone
    - Press button B to show a sad face and play an error tone
    - Press both buttons together to exit the demo
    """
    display.scroll("Press A or B")
    
    while True:
        if button_a.is_pressed() and button_b.is_pressed():
            display.scroll("Exit")
            break
        
        elif button_a.is_pressed():
            show_happy_face()
            play_success_tone()
            sleep(500)
        
        elif button_b.is_pressed():
            show_sad_face()
            play_error_tone()
            sleep(500)

def led_pattern_demo():
    """
    Demonstrate different LED patterns
    
    Instructions:
    - Press A to cycle through different patterns
    - Press B to play a sound with the current pattern
    - Press both buttons together to exit the demo
    """
    display.scroll("LED Demo")
    
    # List of pattern functions
    patterns = [
        led_chase,
        flash_border,
        random_sparkle,
        lambda: display.show(Image.HEART),
        lambda: display.show(Image.GHOST)
    ]
    
    current_pattern = 0
    
    while True:
        # Run the current pattern
        patterns[current_pattern]()
        
        # Check for button presses
        if button_a.is_pressed() and button_b.is_pressed():
            display.scroll("Exit")
            break
        
        elif button_a.is_pressed():
            current_pattern = (current_pattern + 1) % len(patterns)
            sleep(300)  # Debounce
        
        elif button_b.is_pressed():
            play_success_tone()
            sleep(300)  # Debounce

def music_demo():
    """
    Demonstrate playing different melodies on the buzzer
    
    Instructions:
    - Press A to play the next melody
    - Press B to stop the current melody
    - Press both buttons together to exit the demo
    """
    display.scroll("Music Demo")
    
    # List of built-in melodies
    melodies = [
        music.NYAN,
        music.JUMP_UP,
        music.JUMP_DOWN,
        music.POWER_UP,
        music.ENTERTAINER
    ]
    
    current_melody = 0
    
    while True:
        # Display which melody we're on
        display.show(str(current_melody + 1))
        
        if button_a.is_pressed() and button_b.is_pressed():
            music.stop(pin=BUZZER_PIN)
            display.scroll("Exit")
            break
        
        elif button_a.is_pressed():
            current_melody = (current_melody + 1) % len(melodies)
            music.play(melodies[current_melody], pin=BUZZER_PIN)
            sleep(300)  # Debounce
        
        elif button_b.is_pressed():
            music.stop(pin=BUZZER_PIN)
            sleep(300)  # Debounce

def reaction_game():
    """
    A simple reaction time game
    
    Instructions:
    - Wait for the happy face to appear
    - Press A as quickly as possible
    - Your reaction time will be displayed
    - Press B to play again, both buttons to exit
    """
    display.scroll("Reaction Game")
    
    while True:
        display.show(Image.ARROW_S)  # Down arrow means wait
        sleep(random.randint(1000, 5000))  # Random wait time
        
        # Start timing and show the target
        display.show(Image.HAPPY)
        start_time = running_time()
        
        # Wait for button press or timeout
        pressed = False
        while not pressed and running_time() - start_time < 3000:
            if button_a.is_pressed():
                pressed = True
        
        if pressed:
            # Calculate and display reaction time
            reaction_time = running_time() - start_time
            play_success_tone()
            display.scroll(str(reaction_time) + "ms")
        else:
            # Too slow
            play_error_tone()
            display.scroll("Too slow!")
        
        # Check for exit or play again
        display.show("?")
        while True:
            if button_a.is_pressed() and button_b.is_pressed():
                display.scroll("Exit")
                return
            elif button_b.is_pressed():
                break
            sleep(100)

# ------------------ MAIN PROGRAM ------------------

def main():
    """Main program that offers a menu of demos"""
    
    # Initial startup animation
    display.show(Image.HAPPY)
    play_success_tone()
    sleep(1000)
    
    # List of available demos with their names and functions
    demos = [
        ("Buttons", button_demo),
        ("LED Patterns", led_pattern_demo),
        ("Music", music_demo),
        ("Game", reaction_game)
    ]
    
    current_demo = 0
    
    # Main menu loop
    while True:
        # Show the name of the current demo
        display.scroll(demos[current_demo][0])
        
        # Check for button presses
        if button_a.is_pressed():
            # Next demo
            current_demo = (current_demo + 1) % len(demos)
            sleep(300)  # Debounce
        
        elif button_b.is_pressed():
            # Run the selected demo
            display.clear()
            demos[current_demo][1]()  # Execute the demo function
            
            # Return to the menu with a small animation
            led_chase()

# Start the program
if __name__ == "__main__":
    main()
