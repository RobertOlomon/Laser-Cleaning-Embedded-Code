Code for the embedded part of the laser cleaning capstone at UW

## Overview

The current idea is to take in a message over the serial protocol, then move to fuffil the specified requirement and on fufillment send
an ack signal to the rest of the system to let it know you're ready for the next command. 

The system will constantly re-fresh, and move to the propper command. 