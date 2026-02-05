from enum import Enum


class ParamNames(Enum):
    INIT_PROMPT = "init_prompt"
    ROI_UNC_THRESH = "roi_unc_thresh"
    SCENE_SIM_THRESH = "scene_sim_thresh"
    ROI_X_BOUNDS = "roi_x_bounds"
    ROI_Y_BOUNDS = "roi_y_bounds"
