from classes.dataset_extractor import DATASET_extractor
import matplotlib.pyplot as plt
import numpy as np

dataset = DATASET_extractor()

status = dataset.structure_check_all()

if status:
    print('Dataset generated successfully')
    dataset.process_all()

