import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv(path) # Choose path file

pkmn_type_colors = ['#78C850',  # Grass
                    '#F08030',  # Fire
                    '#6890F0',  # Water
                    '#A8B820',  # Bug
                    '#A8A878',  # Normal
                    '#A040A0',  # Poison
                    '#F8D030',  # Electric
                    '#E0C068',  # Ground
                     ]

g=sns.stripplot(data=df, palette=pkmn_type_colors)
g.set_xticklabels(g.get_xticklabels(),rotation=30)

# Set title and axis labels
g.axes.set_title("Signal-Noise ratio of Antennas Without Magnet",
                     fontsize=16)

g.set_xlabel("Antennas",
                 fontsize=14)

g.set_ylabel("Signal Noise Ratio",
                 fontsize=14)
plt.show()
