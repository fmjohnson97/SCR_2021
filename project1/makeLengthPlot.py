from matplotlib import pyplot as plt

objects=[3,4,5,6,7,8]
lengths=[225,262,310,359,310,304]

plt.plot(objects,lengths)
plt.xlabel('Number of Objects')
plt.ylabel('Length of Path')
plt.title('Number of Objects vs Path Length for OMPL and the Lumibot')
plt.show()