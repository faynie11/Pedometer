# Pedemeter using microcontroler FRDM-KL05Z  
 Pedometer allows us to easily count steps. The display shows the total number of steps taken and the number of steps per minute. Since we are somewhat limited by a short USB cable, we need to simulate each step to ensure it is registered. To do this, we have to lift our board, equipped with an accelerometer, upwards, and then shake it downwards.
 It counts your total steps and also steps per min. 
  
 The results are based on readings from the accelerometer. The sum of the absolute values of the 'X,' 'Y,' and 'Z' coordinates is calculated. The data should ideally resemble a sinusoidal pattern for one step. However, due to the lack of appropriate filtering and a low sampling frequency, they more closely resemble peaks. In the program, step detection is performed as follows: a step is detected when two threshold values in the state machine are exceeded - after surpassing a value of 2000, the state machine waits for a value below 1900, approximating the execution of a step. Upon detecting this sequence, one step is added, and the state machine waits for the next occurrence of the sequence. When the samples in the buffer end with a state above 2000, the state machine waits for a drop below 1900, starting the search for the beginning of the next set of data. The chart is attached below. The data was transmitted via UART, and the Python code for the chart is provided."  
   
 You can run program using programming environment Keil uVison
