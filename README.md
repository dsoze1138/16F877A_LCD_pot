16F877A Homework Problem
========================

This project is one possible solution to a homework problem from forum post: 

https://stackoverflow.com/questions/58376434/eet-470-asm-code-designs-adc-conversions-and-voltage-ranges-display-on-a-lcd

The assignment is described as:

Keep everything the same as in Homework #1, design a system that can read the potentiometer’s 
settings and display the real-time voltage ranges of High, Medium, and Low on the LCD module. 
Your program should display the real-time voltage of High=4.5 V and above, Medium=between 2.0V and 3.0V, 
and Low=1.5V and below and update the reading and display every 5 seconds to reflect the changes 
on the potentiometer. If the voltage in not in the ranges, then display “Unknown” on the LCD.

The hardware platform is the: uC Training System Rev. 3 board.

This board was developed as a joint venture between Old Dominion University (ODU, VA), 
Wayne State University (WSU, MI), Bellingham Technical College (BTC, WA), 
Tidewater Community College (TCC, VA) and Blue Ridge Community College (BRCC, VA).

It is used as a training aide in workshops for embedded controller hardware and software development.

Documentation can be found here:

https://ucdistancetraining.org/preworkshpfiles/uC%20Training%20System%20Manual_Rev3_9_28_16.pdf

This repository is an archive of how the Original Posters code from the Stack Overflow topic 
was reworked to meet the requirements of the homework assignment.

For me this is an exercise on using Git to keep track of my changes to a project.

A data sheet for the HD44780 controller that is typical of ones used in many LCD modules can be found here:

https://www.sparkfun.com/datasheets/LCD/HD44780.pdf

To see a log of what has been updated see the notes.txt file in the docs folder.

Well my lack of experience with git has caused me to loose all the history in this repository. 
But at least the project is feature complete. Changes will be tracked from now on.
