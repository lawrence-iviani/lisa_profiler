import pandas as pd
from functions.helpers import _format_header, _process_data, _write_to_excel, _export_all_figures  # data handling
from functions.helpers import plot_boxs, props_1, props_2, props_3, BOX_WIDTH, _show, plot_items_descr  # data plotting

IMPORT_FOLDER = '20202112/raw_data_subjective_validation'
EXPORT_FOLDER = '20202112/raw_data_subjective_validation_exported'
EXPORT_DATA = False  # if false show on screen the pictures

############################
### CONFIGURATION IMPORT ###
############################
# the first N columns are information, starting from ANSWER_FIRST_COL only answer are expected
ANSWER_FIRST_COL = 2
# the row containing the question's text
QUESTION_ROW = 0
EXPERT = "Expert"
NON_EXPERT = "No Expert"

pd_dict = {
            "Without Acoustic":  pd.read_csv(IMPORT_FOLDER + '/no_acoustic_channels.csv'),
            "With Acoustic Channel":  pd.read_csv(IMPORT_FOLDER + '/with_acoustic_channels.csv'),
            "Perceived Improvement":  pd.read_csv(IMPORT_FOLDER + '/improvement perception.csv'),
}

###################
### IMPORT DATA ###
###################
print(_format_header("IMPORT data"))

df_dict = {}
index_expert = []
index_non_expert = []
index_all = []
for k, v in pd_dict.items():
    #print(v.to_string())
    name_tester = list(v.columns[ANSWER_FIRST_COL:])
    # Find the index for the two types of tester
    index_expert = [n for n, name in enumerate(name_tester) if (name.startswith(EXPERT))]
    index_non_expert = [n for n, name in enumerate(name_tester) if (name.startswith(NON_EXPERT))]
    index_all = index_expert + index_non_expert
    # print(index_expert, index_non_expert)
    assert len(name_tester) == len(index_all), "All user has to be mapped"     # + len(index_non_expert), \

    # add \n every characters
    def _format_str(string, every=25):
        lines = []
        for i in range(0, len(string), every):
            lines.append(string[i:i + every])
        return '\n'.join(lines)

    name_question = [_format_str(list(v.iloc[q, :])[0]) for q in range(0, len(v.index))]  # i am sure there is a better wa

    df_dict[k] = v.iloc[:, ANSWER_FIRST_COL:].T  # This is correct for single serie
    df_dict[k] .index = name_tester
    df_dict[k] .columns = name_question


#######################
### DATA PROCESSING ###
#######################


print(_format_header("PROCESSING data"))
df_dict_results = {}
for k, df in df_dict.items():
    df_dict_results[k + '_non_expert'] = _process_data(df, index_expert)
    df_dict_results[k + '_expert'] = _process_data(df, index_non_expert)
    df_dict_results[k + '_both'] = _process_data(df, index_all)


#################
### PLOT DATA ###
#################
print(_format_header("Prepare plots"))

df_results = {}
for k, df in df_dict.items():
    df_non_expert = df.iloc[index_expert]
    df_expert = df.iloc[index_non_expert]
    df_both = df
    plot_descr = plot_items_descr(title=k,
                                  xlabel="Questions",
                                  ylabel="Based on the question\nAgreement Level | N_# items | Frequency\n From 1 to 5",
                                  ylim=[0.5, 5.5],
                                  bkg_color=None)
    plot_boxs([df_non_expert, df_expert, df_both],
              plot_descr,
              [props_1, props_2, props_3],
              ['expert', 'non expert', 'both'],
              [0, BOX_WIDTH, 2*BOX_WIDTH])



###################
### EXPORT DATA ###
###################
print(_format_header("Export data"))

if EXPORT_DATA:
    # export to excel
    with pd.ExcelWriter(EXPORT_FOLDER + '/results_subj_validation.xlsx') as writer:
        _write_to_excel(writer, pd_dict, 'Original')
        _write_to_excel(writer, df_dict, 'Processed')
        _write_to_excel(writer, df_dict_results, 'Results')

    # export all the open figures
    _export_all_figures(EXPORT_FOLDER, "subj_validation")

else:

    for k, df in df_dict_results.items():
        print(_format_header("RESULTS FOR " + k))
        print(df.to_string())
    _show()