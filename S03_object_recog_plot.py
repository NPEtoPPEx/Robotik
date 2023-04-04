# run the code to plot the ground sensor results 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# get data from CSV file
data = pd.read_csv('object_recog.csv', index_col=0)

sns.reset_defaults()
sns.set(
    rc={'figure.figsize':(15,10)}, 
    style="white" # nicer layout
    
)

palette = {
    'Red Block': 'red',
    'Green Block': 'green',
    'Blue Block': 'blue',
    'Black Block': 'black',
    'Black Ball': 'violet',
    'Epuck': 'orange',
}
threshold = [0,0.7,0.8,0.9,0.95,0.97]

for thresh in threshold:


    #Scatter plot general
    ax = sns.scatterplot(
        x='x_center',
        y='y_center',
        data=data[data["conf"] >= thresh],    
        hue='label',
        palette=palette
    )

    ax.invert_yaxis()
    ax.legend(bbox_to_anchor=(1.02,1),loc="upper left",borderaxespad=0)
    ax.set_title("Prediction with threshold = {}".format(thresh))

    plt.ylim(0,120)
    plt.xlim(0,160)
    ax.invert_yaxis()
    save = "prediction_thresh_{}.png".format(thresh)
    # save plot
    plt.savefig(save)
    plt.show()
