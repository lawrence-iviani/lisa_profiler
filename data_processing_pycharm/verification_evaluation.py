import os
import shutil
import pandas as pd
from numpy import linspace
from functions.helpers import _format_header, _process_data_transposed, _process_data

#IMPORT_FOLDER = '20201224/matrix_1mic/'
#EXPORT_FOLDER = '20201224/matrix_1mic_exported/' #'exported'
IMPORT_FOLDER = '20201224/matrix_beamforming/'
EXPORT_FOLDER = '20201224/matrix_beamforming_exported/' #'exported'
IMPORT_FOLDER = '20201224/respeaker_4mic/'
EXPORT_FOLDER = '20201224/respeaker_4mic_exported/' #'exported'

#"20202112/raw_data_verification_test/matrix_fix_speech_level/testset_55db_level_21dec_v2.bag_NOT RECOGNIZED.csv"
EXPORT_DATA = False  # if false show on screen the pictures
#TEST_PREFIX = "testset_spk80_75db_level_21dec_v3" # "testset_spk80_75db_level_21dec_v2" #"testset_quiet_room_21dec_v2" #"testset_70db_level_21dec_v2"  # "testset_quiet_room_21dec_v2"
# TEST_PREFIX = "1mic_spk80_75db_level"
OUTCOME_TO_ANALYSE = ["WAKEDUP",
                      "NOT WAKEDUP",
                      "NOT RECOGNIZED",
                      "RECOGNIZED",
                      "WRONG RECOGNIZED",
                      "TEXT ACQUIRED"]

#TEST_PREFIX = TEST_PREFIX + ".bag"  # this remains from the export of csv
Z_SCORE = 1.96  # 95 percent of the value

###################
### IMPORT DATA ###
###################
print(_format_header("IMPORT data"))


FILE_TO_TEST = []
for file in os.listdir(IMPORT_FOLDER):
    if file.endswith(".bag"):
        FILE_TO_TEST.append(os.path.join(IMPORT_FOLDER, file))

if IMPORT_FOLDER == EXPORT_FOLDER:
    print("you dont wanna do this, trust me....")
    exit(1)
if not os.path.exists(EXPORT_FOLDER):
    os.makedirs(EXPORT_FOLDER)
else:
    if os.path.exists(EXPORT_FOLDER) and os.path.isdir(EXPORT_FOLDER):
        shutil.rmtree(EXPORT_FOLDER)
    os.makedirs(EXPORT_FOLDER)

for TEST_PREFIX in FILE_TO_TEST:
    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print(_format_header(TEST_PREFIX))
    pd_dict = {}
    for o in OUTCOME_TO_ANALYSE:
        #pd_dict[o] = pd.read_csv(IMPORT_FOLDER + '/' + TEST_PREFIX + "_" + o + ".csv")
        pd_dict[o] = pd.read_csv(TEST_PREFIX + "_" + o + ".csv")
        pd_dict[o].index = pd_dict[o].iloc[:, 0] # set the row name as first column and remove it
        pd_dict[o] = pd_dict[o].drop(columns=pd_dict[o].columns[0])
        if not pd_dict[o].empty:
            pd_dict[o] = pd_dict[o].drop(['outcome'])
        pd_dict[o]=pd_dict[o].astype(float)
        #print("\n----------------\n", o)
        #print("COL\n", pd_dict[o].columns)
        #print("ROWS\n", pd_dict[o].index)
        #print(pd_dict[o].to_string())


    #######################
    ### DATA PROCESSING ###
    #######################
    print(_format_header("PROCESSING data"))
    descr_dict = {}
    df_all_dict = {}
    pd_all = pd.DataFrame() # Prepare a df with all the values not aggreagated
    for k, v in pd_dict.items():
        if len(v.columns) and len(v.index):  # empty  df are discarded
            #print("----------------------------------------------")
            #print("key dict", k)
            descr_dict[k] = _process_data(v.T, z_score=Z_SCORE)
            df_all_dict[k] = v.T.dropna()
            pd_all = pd_all.append(v.T)
            #print(descr_dict[k])
            #print("----------------------------------------------")
        else:
            print("Discarding empty df: ", k)

    results_count_dict = {}
    for k, v in pd_dict.items():
        results_count_dict[k] = len(v.columns)
    results_count = pd.DataFrame(results_count_dict, index=[0])
    total_tests = float(results_count.sum(axis=1))
    #results_count = results_count / total_tests
    total_possible_intents =  float(results_count['RECOGNIZED'] + results_count['NOT RECOGNIZED'] + results_count['WRONG RECOGNIZED'] + results_count['TEXT ACQUIRED'])

    if total_possible_intents == 0:
        INTENTS_ER = 1.0
    else:
        INTENTS_ER = float(results_count['NOT RECOGNIZED'] + results_count['WRONG RECOGNIZED'] + results_count['TEXT ACQUIRED'])/total_possible_intents
    if total_tests==0:
        WAKEUP_ER = 1.0
    else:
        WAKEUP_ER = float(results_count['WAKEDUP'] + results_count['NOT WAKEDUP']) / total_tests

    df_all = {}
    df_all_describe = {}
    for c in pd_all.columns:
        s = pd.DataFrame(pd_all[c].dropna())
        df_all[c] = s
        df_all_describe[c] = s.describe()


    df_all_dict["ALL"] = df_all
    #print(df_all_dict)

    #################
    ### PLOT DATA ###
    #################
    print(_format_header("Prepare plots"))

    from functions.helpers import plot_boxs, plot_boxs_2,plot_boxs_3, props_1, props_2, props_3, BOX_WIDTH, _show, plot_items_descr, _export_all_figures_2 , _write_to_excel # data plotting

    plot_descr = plot_items_descr(title="All non aggregated", xlabel="Operation times",
                                  ylabel="Sec.",
                                  ylim=[0.0, 3.0],
                                  bkg_color=None)

    # plot all the data
    props = [props_1, props_2, props_3]
    props = dict(zip(df_all.keys(), props))
    plot_boxs_2(df_all, plot_descr, props)
    plot_descr_2 = plot_items_descr(title="All aggregated", xlabel="Operation times",
                                  ylabel="Sec.",
                                  ylim=[0.0, 3.0],
                                  bkg_color=None)

    # plot the data by key
    props_2 = [props_3, props_2, props_1, props_3]
    #props_2 = dict(zip(descr_dict.keys(), props_2))
    plot_boxs_3(descr_dict,  plot_descr_2, props_2, showmean=False)
    HEIGHT = 9
    sizes = [(5,HEIGHT), (2.5,HEIGHT), (4,HEIGHT), (4,HEIGHT), (3,HEIGHT)]

    print("INTENT ERROR RATE:", INTENTS_ER)
    print("WAKEUP ERROR RATE:", WAKEUP_ER)
    print("TEST PERFORMED:", total_tests)
    print("AVAILABLE INTENTS:", total_possible_intents)
    for k,v in df_all_describe.items():
        print(k, v)

    if EXPORT_DATA:
        fname = os.path.basename(TEST_PREFIX)
        _export_all_figures_2(EXPORT_FOLDER, fname + "_verification_evaluation", size=sizes)
        with pd.ExcelWriter(EXPORT_FOLDER +'/' + fname+'_verification.xlsx') as writer:
            _write_to_excel(writer, descr_dict, '')
            results = pd.DataFrame({'Performed Tests': [total_tests],
                                    'Intents Processed': [total_possible_intents],
                                    'Intent Error Rate': [INTENTS_ER],
                                    'Wake Error Rate': [WAKEUP_ER],})
            _write_to_excel(writer, results, '')

_show()




exit()
for k, df in descr_dict.items():
    plot_descr = plot_items_descr(title=k, xlabel="Operation times",
                                  ylabel="Sec.",
                                  ylim=None,
                                  bkg_color=None)

    plot_boxs([df],
              plot_descr,
              [props_1, props_2, props_3],
              ['boh',],# 'non expert', 'both'],
              [0])#, BOX_WIDTH, 2*BOX_WIDTH])


_show()

