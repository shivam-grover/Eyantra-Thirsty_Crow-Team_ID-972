# Eyantra : Winners-TeamID#972 (Thirsty Crow)

**Winner of eYRC 2018**
Theme Thirsty Crow

## How to use

Open FinalPython.py on any IDE

Connect your XBee to your laptop and change the port number for serial communication

Run the script and start your bot

## Sections : 

## 1.Path Planning 

Algorithm used : Modified version of Breadth first search Algorithm.  
Modification : Saving the predecessor nodes in order to backtrack to source node. 

## 2. Blender Animation : Specified within the final python file.
Threading was used to run the path planning algorithm in background and the animation triggers in the foreground.Textures were overlayed separately on the blender objects to enhance the quality of the animation.Key Frame animation was used to order to overlay animated objects on the aruco markers.   

## 3. Bot Traversal :
Interrupts were used to recieve the path (being sent by the python script as an list of characters). Each character specified a particular movement by the bot.For the ADC interfacing - A relative approach was followed rather than using a  thresholding value for the white line sensor calliberation. 

  
