import numpy as np
import matplotlib.pyplot as plt


def plot_data(figID, data, legends, xlabel, ylabel, save_path):
    plt.figure(figID)
    for idx, label_name in enumerate(legends):
        plt.plot(data[0], data[idx + 1], label=label_name)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.legend(loc="upper left")
    plt.savefig(save_path)
