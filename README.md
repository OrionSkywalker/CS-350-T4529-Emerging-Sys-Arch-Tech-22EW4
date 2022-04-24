# CS-350-T4529-Emerging-Sys-Arch-Tech-22EW4
* Summarize the project and what problem it was solving.
     * For the prototype, you will use the TMP006 temperature sensor to read the room temperature (via I2C), an LED to indicate the output to the thermostat where LED on = heat on (via GPIO), two buttons to increase and decrease the set temperature (via GPIO interrupt), and the UART to simulate the data being sent to the server.


* What did you do particularly well?
     * I think that I made a decent task scheduler instead of resorting to a harder to understand series of counters and if/else logic to get the requested behavior.
* Where could you improve?
     * This is my first exposure to the concept of an exandable task scheduler. I could definitely stand to learn more by making and using them.
* What tools and/or resources are you adding to your support network?
     * Oreilly media is awesome.
* What skills from this project will be particularly transferable to other projects and/or course work?
     * The task scheduler concept. Of course, microcontrollers and input and output devices can solve many problems, but I have experience programming PLC's and a background in electronics so this isn't super interesting to me at the moment. I like coding in C also. 
* How did you make this project maintainable, readable, and adaptable?
     * The task scheduler can be used for many different functions.
