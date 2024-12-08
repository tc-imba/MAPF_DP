import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from numpy.ma.core import shape

from experiment.utils import project_root

map_file = project_root / "maps" / "discrete-random-32-32-0.300000-0-2.map"

with map_file.open() as f:
    lines = f.readlines()
    arr = []
    for i in range(4, len(lines)):
        row = []
        for char in lines[i].strip():
            row.append(0 if char == "." else 1)
        arr.append(row)

arr = np.array(arr)
print(arr.shape)

rgba = np.zeros(shape=(arr.shape[0], arr.shape[1], 4))

for i in range(arr.shape[0]):
    for j in range(arr.shape[1]):
        if arr[i][j] == 1:
            rgba[i][j] = 0, 0, 0, 1
        else:
            rgba[i][j] = 229 / 255, 229 / 255, 229 / 255, 1

# cmap = plt.cm.gray
# norm = plt.Normalize(arr.min(), arr.max())
# rgba = cmap(norm(1 - arr))
#
# print(rgba)

fig = plt.figure()
fig.patch.set_visible(False)
ax = fig.add_subplot(111)

plt.imshow(rgba, interpolation='nearest')

plt.axis('off')
plt.axis("tight")
plt.axis("image")

plt.gca().set_axis_off()
plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0,
                    hspace = 0, wspace = 0)
plt.margins(0,0)
plt.gca().xaxis.set_major_locator(plt.NullLocator())
plt.gca().yaxis.set_major_locator(plt.NullLocator())

# plt.show()
plt.savefig(project_root / "maps" / "discrete-random-32-32-0.300000-0-2.pdf", bbox_inches='tight', pad_inches = 0)
