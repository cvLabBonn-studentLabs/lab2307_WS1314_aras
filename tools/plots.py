#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import pylab as pl

# Image patch size effect
save_to = "/home/neek/Documents/mainf2307/report/pic/"

### HEAD ###


head_recall_21 = np.array([0.754967, 0.731788, 0.695364, 0.642384, 0.619205, 0.60596, 0.592715, 0.559603, 0.523179, 0.483444, 0.440397, 0.410596, 0.380795, 0.36755, 0.331126, 0.281457, 0.235099, 0.152318, 0.0794702])
head_precision_21 = np.array([0.59375, 0.644315, 0.666667, 0.708029, 0.742063, 0.772152, 0.788546, 0.793427, 0.79798, 0.815642, 0.820988, 0.821192, 0.815603, 0.828358, 0.819672, 0.809524, 0.797753, 0.730159, 0.648649])

head_recall_31 = np.array([0.897351, 0.890728, 0.870861, 0.844371, 0.837748, 0.817881, 0.784768, 0.751656, 0.725166, 0.682119, 0.65894, 0.596026, 0.536424, 0.490066, 0.443709, 0.370861, 0.304636, 0.228477, 0.0960265])
head_precision_31 = np.array([0.684343, 0.768571, 0.785075, 0.801887, 0.821429, 0.823333, 0.820069, 0.831502, 0.8327, 0.834008, 0.839662, 0.84507, 0.852632, 0.855491, 0.858974, 0.854962, 0.844037, 0.821429, 0.783784])

head_recall_41 = np.array([0.923841, 0.890728, 0.870861, 0.850993, 0.844371, 0.827815, 0.811258, 0.788079, 0.781457, 0.748344, 0.715232, 0.642384, 0.543046, 0.433775, 0.334437, 0.241722, 0.162252, 0.0496689])
head_precision_41 = np.array([0.687192, 0.741047, 0.78979, 0.810726, 0.819936, 0.833333, 0.841924, 0.853047, 0.867647, 0.869231, 0.874494, 0.869955, 0.863158, 0.86755, 0.87069, 0.890244, 0.890909, 0.882353])

head_recall_51 = np.array([0.857616, 0.817881, 0.788079, 0.778146, 0.764901, 0.758278, 0.738411, 0.711921, 0.68543, 0.652318, 0.60596, 0.523179, 0.453642, 0.397351, 0.324503, 0.258278, 0.178808, 0.115894, 0.0264901])
head_precision_51 = np.array([0.809375, 0.817881, 0.835088, 0.842294, 0.852399, 0.874046, 0.881423, 0.881148, 0.884615, 0.899543, 0.897059, 0.902857, 0.913333, 0.944882, 0.970297, 0.987342, 1.0, 1.0, 1.0])

head_recall_61 = np.array([0.956954, 0.956954, 0.956954, 0.956954, 0.953642, 0.94702, 0.927152, 0.92053, 0.917219, 0.907285, 0.89404, 0.874172, 0.844371, 0.827815, 0.794702, 0.745033, 0.665563, 0.430464, 0.145695])
head_precision_61 = np.array([0.558994, 0.621505, 0.647982, 0.678404, 0.695652, 0.715, 0.732984, 0.753388, 0.771588, 0.782857, 0.791789, 0.814815, 0.852843, 0.87108, 0.869565, 0.9, 0.926267, 0.948905, 0.956522])
"""
head_figure = plt.figure()
ax_head = head_figure.add_subplot(111)

ax_head.plot(head_recall_21, head_precision_21, 'c--', label='21x21', linewidth=2)
ax_head.plot(head_recall_31, head_precision_31, 'r-.', label='31x31', linewidth=2)
ax_head.plot(head_recall_41, head_precision_41, 'g-o', label='41x41', linewidth=2)
ax_head.plot(head_recall_51, head_precision_51, 'm*-', label='51x51', linewidth=2)
ax_head.plot(head_recall_61, head_precision_61, 'b-v', label='61x61', linewidth=2)

plt.xlabel('Recall')
plt.ylabel('Precision')
plt.axis([0, 1, 0, 1.1])
plt.legend(loc=3)
plt.title('Head')
plt.savefig(save_to + "PR_head.png")
plt.show()

"""
### HANDS ###
hands_recall_21 = np.array([0.145695, 0.130795, 0.10596, 0.0910596, 0.0778146, 0.0629139, 0.0529801, 0.0413907, 0.0397351, 0.0347682, 0.031457, 0.0281457, 0.0231788, 0.0165563, 0.0165563, 0.013245, 0.00993378, 0.00827815, 0.00496689])
hands_precision_21 = np.array([0.0555205, 0.0646481, 0.0675818, 0.0706033, 0.0737834, 0.0701107, 0.0695652, 0.0641026, 0.0745342, 0.0780669, 0.0829694, 0.0894737, 0.0927152, 0.0840336, 0.102041, 0.103896, 0.111111, 0.131579, 0.214286])

hands_recall_31 = np.array([0.271523, 0.256623, 0.231788, 0.216887, 0.19702, 0.173841, 0.153974, 0.139073, 0.125828, 0.107616, 0.0877483, 0.0728477, 0.0645695, 0.0562914, 0.044702, 0.0364238, 0.0231788, 0.0198676, 0.0115894])
hands_precision_31 = np.array([0.216931, 0.24181, 0.24735, 0.259921, 0.263858, 0.261194, 0.255495, 0.251497, 0.252492, 0.249042, 0.23348, 0.222222, 0.233533, 0.223684, 0.216, 0.205607, 0.159091, 0.181818, 0.233333])

hands_recall_41 = np.array([0.415563, 0.410596, 0.402318, 0.395695, 0.389073, 0.387417, 0.387417, 0.377483, 0.375828, 0.364238, 0.352649, 0.34106, 0.322848, 0.299669, 0.279801, 0.246689, 0.201987, 0.165563, 0.089404])
hands_precision_41 = np.array([0.316121, 0.349788, 0.365964, 0.376378, 0.38715, 0.397284, 0.404844, 0.409336, 0.415751, 0.419847, 0.424303, 0.42562, 0.422078, 0.421911, 0.423559, 0.428161, 0.431095, 0.46729, 0.418605])

hands_recall_51 = np.array([0.541391, 0.466887, 0.370861, 0.299669, 0.226821, 0.190397, 0.152318, 0.13245, 0.112583, 0.0943709, 0.0778146, 0.0612583, 0.0562914, 0.0413907, 0.0331126, 0.0264901, 0.0165563, 0.0149007])
hands_precision_51 = np.array([0.536066, 0.57551, 0.586387, 0.605351, 0.600877, 0.614973, 0.601307, 0.601504, 0.607143, 0.612903, 0.580247, 0.536232, 0.557377, 0.531915, 0.526316, 0.592593, 0.588235, 0.692308])

hands_recall_61 = np.array([0.688742, 0.672185, 0.668874, 0.667219, 0.660596, 0.65894, 0.652318, 0.647351, 0.64404, 0.642384, 0.637417, 0.63245, 0.619205, 0.586093, 0.539735, 0.471854, 0.374172, 0.206954, 0.0860927])
hands_precision_61 = np.array([0.339315, 0.386667, 0.423924, 0.456399, 0.477844, 0.501259, 0.528859, 0.555398, 0.568713, 0.581709, 0.595054, 0.612179, 0.639316, 0.661682, 0.676349, 0.680191, 0.697531, 0.675676, 0.712329])
"""
hands_figure = plt.figure()
ax_hands = hands_figure.add_subplot(111)

ax_hands.plot(hands_recall_21, hands_precision_21, 'c--', label='21x21', linewidth=2)
ax_hands.plot(hands_recall_31, hands_precision_31, 'r-.', label='31x31', linewidth=2)
ax_hands.plot(hands_recall_41, hands_precision_41, 'g-o', label='41x41', linewidth=2)
ax_hands.plot(hands_recall_51, hands_precision_51, 'm*-', label='51x51', linewidth=2)
ax_hands.plot(hands_recall_61, hands_precision_61, 'b-v', label='61x61', linewidth=2)

plt.xlabel('Recall')
plt.ylabel('Precision')
plt.axis([0, 1, 0, 1.1])
plt.legend(loc=4)
plt.title('Hands')
plt.savefig(save_to + "PR_hands.png")
plt.show()

"""
### FEET ###
feet_recall_21 = np.array([0.00496689, 0.00165563, 0.00165563, 0.00165563])
feet_precision_21 = np.array([0.25, 0.111111, 0.2, 0.2])

feet_recall_31 = np.array([0.0182119, 0.00993378, 0.00827815, 0.00827815, 0.00662252, 0.00496689, 0.00496689, 0.00496689, 0.00496689, 0.00496689, 0.00496689, 0.00496689, 0.00331126, 0.00331126, 0.00331126, 0.00331126, 0.00331126, 0.00165563])
feet_precision_31 = np.array([0.23913, 0.230769, 0.25, 0.294118, 0.285714, 0.272727, 0.3, 0.333333, 0.428571, 0.428571, 0.428571, 0.428571, 0.333333, 0.4, 0.4, 0.4, 0.666667, 0.5])

feet_recall_41 = np.array([0.0231788, 0.0165563, 0.013245, 0.00993378, 0.00827815, 0.00827815, 0.00827815, 0.00496689, 0.00331126])
feet_precision_41 = np.array([0.215385, 0.217391, 0.205128, 0.171429, 0.15625, 0.166667, 0.172414, 0.130435, 0.0952381])

feet_recall_51 = np.array([])
feet_precision_51 = np.array([])

feet_recall_61 = np.array([0.013245, 0.0115894, 0.00827815, 0.00827815, 0.00662252, 0.00662252, 0.00662252, 0.00496689, 0.00496689, 0.00331126, 0.00331126, 0.00331126, 0.00165563])
feet_precision_61 = np.array([0.181818, 0.184211, 0.15625, 0.16129, 0.137931, 0.148148, 0.166667, 0.15, 0.1875, 0.166667, 0.2, 0.222222, 0.142857])
"""
feet_figure = plt.figure()
ax_feet = feet_figure.add_subplot(111)

ax_feet.plot(feet_recall_21, feet_precision_21, 'c--', label='21x21', linewidth=2)
ax_feet.plot(feet_recall_31, feet_precision_31, 'r-.', label='31x31', linewidth=2)
ax_feet.plot(feet_recall_41, feet_precision_41, 'g-o', label='41x41', linewidth=2)
ax_feet.plot(feet_recall_51, feet_precision_51, 'm*-', label='51x51', linewidth=2)
ax_feet.plot(feet_recall_61, feet_precision_61, 'b-v', label='61x61', linewidth=2)

plt.xlabel('Recall')
plt.ylabel('Precision')
plt.axis([0, 0.03, 0, 0.8])
plt.legend(loc=1)
plt.title('Feet')
plt.savefig(save_to + "PR_feet.png")
plt.show()
"""


"""
### Uniform centroid distribution ###
head_recall_41_uniform = np.array([0.850993, 0.860927, 0.831126, 0.794702, 0.827815, 0.81457, 0.801324, 0.781457, 0.758278, 0.731788, 0.672185, 0.629139, 0.559603, 0.476821, 0.413907, 0.380795, 0.251656, 0.115894, 0.0165563])
head_precision_41_uniform = np.array([0.47417, 0.594966, 0.664021, 0.705882, 0.735294, 0.732143, 0.773163, 0.810997, 0.841912, 0.840304, 0.849372, 0.87963, 0.875648, 0.9, 0.932836, 0.950413, 0.974359, 0.99, 0.99])

hands_recall_41_uniform = np.array([0.511513, 0.519868, 0.483444, 0.5, 0.478477, 0.480132, 0.456954, 0.445364, 0.450331, 0.465232, 0.438742, 0.40894, 0.410596, 0.390728, 0.352649, 0.32947, 0.263245, 0.22351, 0.135762])
hands_precision_41_uniform = np.array([0.210991, 0.227536, 0.231196, 0.246732, 0.255075, 0.263158, 0.253676, 0.271443, 0.288136, 0.294858, 0.297419, 0.287879, 0.309613, 0.312169, 0.325688, 0.347902, 0.327161, 0.359043, 0.392345])

feet_recall_41_uniform = np.array([0.0627063, 0.0572831, 0.044335, 0.0378289, 0.0296053, 0.0377049, 0.0392799, 0.0346535, 0.0197368, 0.0377049, 0.029654, 0.0328947, 0.0181518, 0.0345395, 0.0165017, 0.0181818, 0.0181518, 0.013245, 0.00331126])
feet_precision_41_uniform = np.array([0.319328, 0.380435, 0.296703, 0.338235, 0.3, 0.396552, 0.510638, 0.446809, 0.342857, 0.605263, 0.642857, 0.740741, 0.5, 0.724138, 0.714286, 0.733333, 0.785714, 0.8, 0.666667])

head_figure = plt.figure()
ax_uniform_head = head_figure.add_subplot(111)

ax_uniform_head.plot(head_recall_41_uniform, head_precision_41_uniform, 'b--', label='w/o centroids', linewidth=2)
ax_uniform_head.plot(head_recall_41, head_precision_41, 'r-.', label='with centroids', linewidth=2)

plt.xlabel('Recall')
plt.ylabel('Precision')
plt.axis([0, 1.0, 0, 1.0])
plt.legend(loc=3)
plt.title('Head')
plt.savefig(save_to + "PR_head_uniform.png")
plt.show()

hands_figure = plt.figure()
ax_uniform_hands = hands_figure.add_subplot(111)

ax_uniform_hands.plot(hands_recall_41_uniform, hands_precision_41_uniform, 'b--', label='w/o centroids', linewidth=2)
ax_uniform_hands.plot(hands_recall_41, hands_precision_41, 'r-.', label='with centroids', linewidth=2)

plt.xlabel('Recall')
plt.ylabel('Precision')
plt.axis([0, .6, 0, .6])
plt.legend(loc=1)
plt.title('Hands')
plt.savefig(save_to + "PR_hands_uniform.png")
plt.show()

feet_figure = plt.figure()
ax_uniform_hands = feet_figure.add_subplot(111)

ax_uniform_hands.plot(feet_recall_41_uniform, feet_precision_41_uniform, 'b--', label='w/o centroids', linewidth=2)
ax_uniform_hands.plot(feet_recall_41, feet_precision_41, 'r-.', label='with centroids', linewidth=2)

plt.xlabel('Recall')
plt.ylabel('Precision')
plt.axis([0, .1, 0, 0.9])
plt.legend(loc=1)
plt.title('Feet')
plt.savefig(save_to + "PR_feet_uniform.png")
plt.show()
"""


### Comparison to the Original ###
head_recall_41_original = np.array([0.828172, 0.806988, 0.801997, 0.777038, 0.768719, 0.787022, 0.75208, 0.745424, 0.694306, 0.751249, 0.735441, 0.725458, 0.668885, 0.690516, 0.663894, 0.58569, 0.489185])
head_precision_41_original = np.array([0.95397, 0.952849, 0.977688, 1, 1, 0.991614, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])

hands_recall_41_original = np.array([0.596404, 0.596632, 0.578841, 0.574814, 0.598009, 0.588715, 0.556701, 0.579646, 0.572514, 0.532661, 0.517733, 0.50071, 0.483501, 0.481107, 0.443874, 0.417476, 0.387509, 0.330422])
hands_precision_41_original = np.array([0.352955, 0.378123, 0.373478, 0.382605, 0.40911, 0.418822, 0.405608, 0.431936, 0.435453, 0.411526, 0.430654, 0.422662, 0.425774, 0.452986, 0.427864, 0.430331, 0.458438, 0.467078])

feet_recall_41_original = np.array([0.609463, 0.583302, 0.577419, 0.569948, 0.536424, 0.516756, 0.534437, 0.478231, 0.462912, 0.461793, 0.403397, 0.401008, 0.372807, 0.300435, 0.306428, 0.262481, 0.18855, 0.133753])
feet_precision_41_original = np.array([0.430276, 0.493389, 0.547401, 0.584329, 0.568421, 0.594449, 0.66971, 0.642009, 0.660137, 0.677165, 0.725191, 0.755767, 0.779817, 0.732743, 0.803922, 0.820331, 0.796774, 0.805687])


head_figure = plt.figure()
ax_uniform_head = head_figure.add_subplot(111)

ax_uniform_head.plot(head_recall_41_original, head_precision_41_original, 'r--', label='AGEX head', linewidth=3)
ax_uniform_head.plot(hands_recall_41_original, hands_precision_41_original, 'g--', label='AGEX hand', linewidth=3)
ax_uniform_head.plot(feet_recall_41_original, feet_precision_41_original, 'b--', label='AGEX foot', linewidth=3)

plt.xlabel('Recall')
plt.ylabel('Precision')
plt.axis([0, 1.0, 0, 1.05])
plt.legend(loc=3)
plt.title('Head')
plt.savefig(save_to + "PR_original.png")
plt.show()
"""


### Confusion matrix for conf_value = 0.65 ###

conf_matrix = np.array([[75.125,0,0,24.875],
                       [2.8,50.138,0.03,46.704],
                       [2.1492,2.352,38.524,56.975]])

# Show confusion matrix in a separate window
pl.matshow(conf_matrix)
pl.title('Confusion matrix (per cent)')
pl.colorbar()
pl.ylabel('True label')
pl.xlabel('Predicted label')
pl.savefig(save_to + "conf_matrix.png")
pl.show()
"""

