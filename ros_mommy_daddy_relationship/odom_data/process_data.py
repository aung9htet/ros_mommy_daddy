import numpy as np
import matplotlib.pyplot as plt

def process_data():
    prefix = "_odom_data_"
    posfix1 = "secure_1_hour_diff"
    posfix2 = "secure_1_hour_same"
    posfix_amb = "ambivalent_1_hour"
    posfix_av = "avoidant_1_hour"
    # file = input("Please type in the file name to process: ") # request input from user of file name
    file_c = "child" + prefix + posfix_av + ".npy"
    file_p = "parent" + prefix + posfix_av + ".npy"
    X_child = np.load(file_c) # will return an np array of tuples that is of (pos_x, pos_y)
    X_parent = np.load(file_p)

    fig, ax = plt.subplots()
    # plt.ion()
    
    for i in range(1,len(X_child)-1):
        ax.plot(X_child[i-1:i+1,0], X_child[i-1:i+1,1], 'b', linewidth = 2.0 )
        ax.plot(X_parent[i-1:i+1,0], X_parent[i-1:i+1,1], 'r', linewidth = 2.0 )
        ax.axis([-10, 10, -10, 10])
        # plt.pause(0.01)
    
    plt.show()

if __name__ == "__main__":
    process_data()