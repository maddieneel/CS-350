# CS-350
Emerging Sys Arch &amp; Tech


# Summarize the project and what problem it was solving.
For Module Three project, I had to design a state machine to turn on an LED when a user types ON as well as turn the LED off when a user inputs OFF into the console. Furthermore, it had to only receive one character at a time from the UART.

For Module Seven project, I had to create a smart thermostat using a TI board. For the prototype, I used a temperature sensor via I2C, used the boards LED lights to indicate if heat is on or off via GPIO, used a GPIO interrupt to increase or decrease the set temperature from two of the user input buttons on the board, and used UART to simulate data being sent to the server. 

# What did you do particularly well?
I felt fairly comfortable and happy with my state machines in both projects. Much of the development of state machines consisted of trial and error to ensure the inputs completed the intended functions. 

# Where could you improve?
One aspect I felt that I should have improved is in the Project Three switch-case, input. While only including capital letter inputs was alright for personal use as I knew what was needed, it is an assumption that other users would have known that capital letters was needed – hence I should have given the option of lower-case as well. 

# What tools and/or resources are you adding to your support network?
As I have not worked with TI boards previously, I now have a general understanding of some of the tasks I can make it do –  especially with different drivers that can be added; i.e. I2C, timer, PWM, GPIO, and more. 

# What skills from this project will be particularly transferable to other projects and/or course work?
There’s many skills from this project that could be transferable to other projects.  This includes the knowledge of task schedulers, working with timers, GPIO interrupts, state machines, switch-cases to implement the machine, and more. Overall, many of these different skills could be modified to fit another project. 

# How did you make this project maintainable, readable, and adaptable?
To help make this project maintainable and readable, I included a fair amount of comments. I avoided using abbreviations that could be misunderstood, explained what each line did while not over-commenting, and included the ZyBooks chapter/figure to help reference where the base of the code block had come from. Otherwise, I also clumped similar code blocks together so I could find something easily by knowing that lines responsibility (such as global variables with other global variables, etc.). 
![image](https://user-images.githubusercontent.com/73367317/164950678-7db18d27-6d1e-4418-9efc-c6f1c04d7c41.png)
