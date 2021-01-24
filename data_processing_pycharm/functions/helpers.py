import pandas as pd
import matplotlib.pyplot as plt


# PLOT DEFINITIONS
BOX_WIDTH = 0.25
LINE_WIDTH = 1.3
ROTATION = 35
ROTATION_2 = 80
FONT_SIZE = 9
SHOWFLIERS = False
SHOWMEAN = True
BACKGROUND_COLOR = (0.95, 0.93, 0.91)
LIGHT_GREY = (0.91, 0.91, 0.91)
DARK_GREY = (0.73, 0.73, 0.73,0.5)
COLOR_1 = '#19AADE'
COLOR_2 = '#AF48CE'
COLOR_3 = '#DE542C'
COLOR_3 = '#DE542C'
COLOR_MEDIAN = '#142459'

props_1 = dict(
    color=dict(boxes=COLOR_1, whiskers=COLOR_1, medians=COLOR_MEDIAN, caps=COLOR_1),
    boxprops=dict(linestyle='-', linewidth=2*LINE_WIDTH,),
    flierprops=dict(linestyle='-', linewidth=LINE_WIDTH),
    medianprops=dict(linestyle='-', linewidth=2*LINE_WIDTH),
    whiskerprops=dict(linestyle='-', linewidth=LINE_WIDTH),
    capprops=dict(linestyle='-', linewidth=LINE_WIDTH, ),
    meanprops=dict(linestyle='--',  marker="X", #markersize=8*LINE_WIDTH,
                   markerfacecolor=LIGHT_GREY, markeredgecolor=COLOR_MEDIAN),
)
props_2 = dict(props_1)
props_2['color'] = dict(boxes=COLOR_2, whiskers=COLOR_2, medians=COLOR_MEDIAN, caps=COLOR_2)

props_3 = dict(props_1)
props_3['color'] = dict(boxes=COLOR_3, whiskers=COLOR_3, medians=COLOR_MEDIAN, caps=COLOR_3)


def _format_header(msg, level = 0):
    if level == 0:
        msg = "----------- " + str(msg) + " -----------"
        line = "-"*len(msg)
        return line + "\n" + msg + "\n" + line
    else:
        return ("===== {} =====".format(msg))

from  scipy.stats import zscore
import numpy as np
def _process_data(df, index=None, z_score=None):
    if index is None:
        df_indexed = df
    else:
        df_indexed = df.iloc[index]
    if z_score is not None:
        abs_z_scores = np.abs(zscore(df_indexed))
        df_indexed = df_indexed[(abs_z_scores < z_score).all(axis=1)]

    df_descr = df_indexed.describe()
    df_with_median = pd.DataFrame(df_indexed.median(), ).T
    df_with_median.index = ['median']

    return df_descr
#    return df_descr.append(df_with_median)


def _process_data_transposed(df, index=None):
    """
    Equivalent of _process_data, for transpos matrixes... More a test. Dont ues it
    :param df:
    :param index:
    :return:
    """
    if index is None:
        df_indexed = df
    else:
        df_indexed = df.iloc[index]

    df_descr = df_indexed.T.describe()  # get the data quartile and mean
    df_with_median = pd.DataFrame(df_indexed.T.median())  # prepare median
    df_with_median.index = df_descr.columns
    df_with_median.columns = ['median']

    return df_descr.append(df_with_median.T)


# export to excel stuff
def _write_to_excel(writer, df_dict, tag):
    for k, df in df_dict.items():
        sheet_name = tag + '_' + k
        print("saving " + sheet_name)
        df.to_excel(writer, sheet_name=sheet_name)

from collections import namedtuple

plot_items_descr = namedtuple('plot_items_descr', ['title', 'xlabel', 'ylabel', 'ylim', 'bkg_color'])

## PLOT STUFF

def plot_boxs_3(dfs, plot_items, props, showmean=SHOWMEAN, rotation=ROTATION_2):
    ax = None
    #idx_grp = keys
    #shift = list(range(len(idx_grp)))
    #shift = dict(zip(idx_grp, shift))

    #labels = idx_grp
    #props = [props_1, props_2, props_3, props_1]
    #props = dict(zip(dfs.keys(), props))
    shift = dict(zip(dfs.keys(), list(range(len(props)))))

    for n, (k, df) in enumerate(dfs.items()):
        col = list(df.columns)
       # s = [shift[s] for s in col]
        p = props[n]
        # print(p)
        s = shift[k]
       # c = colors[:range(len(col))]
      #  print(col, c)
        #if ax is None:
            #ax = df.plot(kind='box', by=idx_grp, widths=BOX_WIDTH, patch_artist=True, positions=s,  # labels=col,

        ax = df.plot(kind='box', by=list(dfs.keys()), widths=BOX_WIDTH, patch_artist=True,return_type='both',# positions=[s],# labels=col,
                     showfliers=SHOWFLIERS, grid=True, rot=rotation, fontsize=FONT_SIZE, showmeans=showmean,
                     sharex=True,
                     color=COLOR_MEDIAN,#p['color']'',
                     boxprops=p['boxprops'],
                     flierprops=p['flierprops'],
                     medianprops=p['medianprops'],
                     whiskerprops=p['whiskerprops'],
                     capprops=p['capprops'],
                     meanprops=p['meanprops'])


        # colors = p['color']
        # for nn , (patch, color) in enumerate(zip(ax[1]['boxes'], colors)):
        #     patch.set_facecolor(colors['boxes'])
        #     patch.set_edgecolor(colors['boxes'])

        colors = props #p['color']
        for nn, (patch, color) in enumerate(zip(ax[1]['boxes'], colors)):
            patch.set_facecolor(colors[nn]['color']['boxes'])
            patch.set_edgecolor(colors[nn]['color']['boxes'])

        # for n, (line, color) in enumerate(zip(ax[1]['whiskers'], colors)):
        #     line.set_color(colors[n]['whiskers'])
        # for n, (line, color) in enumerate(zip(ax[1]['caps'], colors)):
        #     line.set_color(colors[n]['caps'])
      #  for n, (line, color) in enumerate(zip(ax[1]['whiskers'], colors)):
      #      line.set_color(colors[n]['whiskers'])
        # box = plt.boxplot(df, patch_artist=True)
        #
        # colors = ['blue', 'green', 'purple', 'tan', 'pink', 'red']
        #
        # for patch, color in zip(ax['boxes'], colors):
        #     patch.set_facecolor(color)

        # else:
        #     ax = df.plot(kind='box', widths=BOX_WIDTH, patch_artist=True, #positions=s, labels=col,
        #                  # positions=shift, labels=label,
        #                  showfliers=SHOWFLIERS, grid=True, rot=rotation, fontsize=FONT_SIZE, showmeans=showmean,
        #                  sharex=True,
        #                  color=p['color'],
        #                  boxprops=p['boxprops'],
        #                  flierprops=p['flierprops'],
        #                  medianprops=p['medianprops'],
        #                  whiskerprops=p['whiskerprops'],
        #                  capprops=p['capprops'],
        #                  meanprops=p['meanprops'])
        _set_plot_items(ax[0], plot_items)
        plt.suptitle(plot_items.title + " - " + k)
    #ax.set_xticklabels(labels)
    return ax

def plot_boxs_2(dfs, plot_items, props, showmean=SHOWMEAN, rotation=ROTATION_2): #, plot_items, props_plot, labels, shifts, ):
    ax = None
    idx_grp = list(dfs.keys())
    shift = list(range(len(idx_grp)))
    shift = dict(zip(idx_grp, shift))

    labels = idx_grp
    # props = [props_1, props_2, props_3]
    # props = dict(zip(idx_grp, props))

    for k, df in dfs.items():
        col = list(df.columns)
        s = [shift[s] for s in col]
        p = props[k]

        if ax is None:
            ax = df.plot(kind='box', by=idx_grp, widths=BOX_WIDTH,  patch_artist=True,  positions=s,# labels=col,
                         showfliers=SHOWFLIERS, grid=True, rot=rotation, fontsize=FONT_SIZE, showmeans=showmean, sharex = True,
                         color=p['color'],
                         boxprops=p['boxprops'],
                         flierprops=p['flierprops'],
                         medianprops=p['medianprops'],
                         whiskerprops=p['whiskerprops'],
                         capprops=p['capprops'],
                         meanprops=p['meanprops'])
        else:
            ax = df.plot(kind='box',  ax=ax, by=idx_grp,  widths=BOX_WIDTH,  patch_artist=True, positions=s, labels=col, #  positions=shift, labels=label,
                         showfliers=SHOWFLIERS, grid=True, rot=rotation, fontsize=FONT_SIZE, showmeans=showmean, sharex = True,
                         color=p['color'],
                         boxprops=p['boxprops'],
                         flierprops=p['flierprops'],
                         medianprops=p['medianprops'],
                         whiskerprops=p['whiskerprops'],
                         capprops=p['capprops'],
                         meanprops=p['meanprops'])
        _set_plot_items(ax, plot_items)
        
    ax.set_xticklabels(labels)
    return ax

def plot_boxs(dfs, plot_items, props_plot, labels, shifts, ):
    """"
    :parameter dfs a list of data frames
    :parameter title unique title
    :parameter props_plot plot prpoperties of every props_plot (must be same length of dfs)
    :parameter labels not used at moment (must be same length of dfs)
    :parameter shifts .a shift on the x axis, used to align plots. (must be same length of dfs)
    """
   # idx_grp = dfs[0].columns # all the dfs has the same column structure
    ax = None
    for n, df in enumerate(dfs):
        idx_grp = df.columns  # all the dfs has the same column structure
        props = props_plot[n]
        if isinstance(props, list):
            props = props_plot[n][0]
        else:
            props = props_plot[n]

        label = [labels[n] for x in range(0,len(idx_grp))]

        _sft = shifts[n]
        if isinstance(_sft, list):
            shift = shifts[n]
        else:
            shift = [1.5 * x + 1.25*shifts[n] for x in range(0, len(idx_grp))]

        # print(label, shift)
        if ax is None:
            ax = df.plot(kind='box', by=idx_grp, widths=BOX_WIDTH,  positions=shift,  patch_artist=True, #labels=label,
                         showfliers=SHOWFLIERS, grid=True, rot=ROTATION, fontsize=FONT_SIZE, showmeans=SHOWMEAN,
                         color=props['color'],
                         boxprops=props['boxprops'],
                         flierprops=props['flierprops'],
                         medianprops=props['medianprops'],
                         whiskerprops=props['whiskerprops'],
                         capprops=props['capprops'],
                         meanprops=props['meanprops'])
        else:
            ax = df.plot(kind='box',  ax=ax, by=idx_grp,  widths=BOX_WIDTH, positions=shift,  patch_artist=True,# labels=labels,# positions=list(range(0,len(idx_grp))),
                         showfliers=SHOWFLIERS, grid=True, rot=ROTATION, fontsize=FONT_SIZE, showmeans=SHOWMEAN,
                         color=props['color'],
                         boxprops=props['boxprops'],
                         flierprops=props['flierprops'],
                         medianprops=props['medianprops'],
                         whiskerprops=props['whiskerprops'],
                         capprops=props['capprops'],
                         meanprops=props['meanprops'] )
        ax.grid(True, axis='x', linestyle='--', linewidth=0.5, color='k', which = 'minor' )
        ax.grid(True, axis='y',  linestyle='--', linewidth=0.75, color='grey')


        _set_plot_items(ax, plot_items)
        # ax.set_xlabel(plot_items.xlabel )#"Questions")
        # ax.set_ylabel(plot_items.ylabel) #'Agreement Level  --- N_# items --- Frequency\n1..5')
        # if plot_items.ylim is not None:
        #     ax.set_ylim(plot_items.ylim[0], plot_items.ylim[1])

    # plt.suptitle(plot_items.title)
    return ax

def _set_plot_items(ax, plot_items):
    ax.set_xlabel(plot_items.xlabel)
    ax.set_ylabel(plot_items.ylabel)
    if plot_items.ylim is not None:
        ax.set_ylim(plot_items.ylim[0], plot_items.ylim[1])
    if plot_items.bkg_color is None:
        ax.set_facecolor(BACKGROUND_COLOR)
    else:
        ax.set_facecolor(plot_items.bkg_color)
    plt.suptitle(plot_items.title)


def _show():
    plt.show()




def _export_all_figures(export_folder, prefix_name, width=18.5, height=10.5):

    for i in plt.get_fignums():
        plt.figure(i, facecolor='k', edgecolor='k')
        plt.figure(i).set_facecolor(LIGHT_GREY)
        plt.figure(i).set_size_inches(width, height, forward=True)
        plt.savefig(export_folder + '/' + prefix_name + '_%d.png' % i, facecolor=plt.figure(i).get_facecolor(),
                    edgecolor='none',
                    bbox_inches='tight', )  # pad_inches=0)


def _export_all_figures_2(export_folder, prefix_name, size):# width=18.5, height=10.5):
    for n, i in enumerate(plt.get_fignums()):
        (width, height) = size[n]
        plt.figure(i, facecolor='k', edgecolor='k')
        plt.figure(i).set_facecolor(LIGHT_GREY)
        plt.figure(i).set_size_inches(width, height, forward=True)
        plt.savefig(export_folder + '/' + prefix_name + '_%d.png' % i, facecolor=plt.figure(i).get_facecolor(),
                    edgecolor='none',
                    bbox_inches='tight', )  # pad_inches=0)
    plt.close('all')