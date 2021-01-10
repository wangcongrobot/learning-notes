# How to use matplotlib

- [how to changing plot scale by a factor in matplotlib](https://stackoverflow.com/questions/10171618/changing-plot-scale-by-a-factor-in-matplotlib)

```python
import matplotlib.ticker as ticker
scale_x = 1e6
ticks_x = ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x/scale_x))
ax0.xaxis.set_major_formatter(ticks_x)
```

- [engineering_formatter](https://matplotlib.org/3.1.0/gallery/text_labels_and_annotations/engineering_formatter.html)

```python
from matplotlib.ticker import EngFormatter
# Demo of the options `places` (number of digit after decimal point) and
# `sep` (separator between the number and the prefix/unit).
ax1.set_title('SI-prefix only ticklabels, 1-digit precision & '
              'thin space separator')
formatter1 = EngFormatter(places=1, sep="\N{THIN SPACE}")  # U+2009
ax1.xaxis.set_major_formatter(formatter1)
ax1.plot(xs, ys)
ax1.set_xlabel('Frequency [Hz]')
```