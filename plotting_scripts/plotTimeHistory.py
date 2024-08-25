import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import numpy as np


def plotTimeHistory(timeArray: list, qtyOfInterest: list, title: str, xlabel: str, ylabel: str, figName: str):

    sns.set(style="ticks", font="Arial", font_scale=1.4)
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(timeArray, qtyOfInterest)

    ax.set(xlabel=xlabel, ylabel=ylabel, title=title)
    ax.grid(alpha=0.2)

    fig.savefig("../plots/" + figName + ".png")
    plt.show()


def plotTimeHistoryMultiple(timeArray: list, qtyOfInterest: dict, title: str, xlabel: str, ylabel: str, figName: str):

    sns.set(style="ticks", font_scale=1.4)
    fig, ax = plt.subplots(figsize=(8, 8))
    
    for label, values in qtyOfInterest.items():
        ax.plot(timeArray, values, label=label)
    
    ax.set(xlabel=xlabel, ylabel=ylabel, title=title)
    ax.grid(alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig("../plots/" + figName + ".png", dpi=600)
    plt.show()

    
