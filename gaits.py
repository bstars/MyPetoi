import numpy as np

standing_gait = np.array([
    # frame 0
    [
        [-5., -5],
        [5, -5],
        [5, -5],
        [-5, -5]
    ]
])

stepping_gait = np.array([
    [ # frame 0
        # foot 0    foot 1      foot 2      foot 3
        [-5, -5],   [5, -6],    [5, -5],    [-5, -6]
    ],
    [ # frame 1
        [-5, -5.5], [5, -5.5],  [5, -5.5],  [-5, -5.5] 
    ],
    [ # frame 2
        [-5, -6],   [5, -5],    [5, -6],    [-5, -5]
    ],
    [ # frame 3
        [-5, -5.5], [5, -5.5],  [5, -5.5],  [-5, -5.5] 
    ]
])

walking_gait = np.array([
    [ # frame 0
        # foot 0    foot 1      foot 2      foot 3
        [-5, -5], [6, -5.75], [5, -5],  [-4, -5.75]
    ],
    [ # frame 1
        [-6, -5.25], [7, -6], [4, -5.25], [-3, -6]
    ],
    [ # frame 2
        [-5, -5.5], [6, -5], [5, -5.5], [-4, -5]
    ],
    [ # frame 3
        [-4, -5.75], [5, -5], [6, -5.75], [-5, -5]
    ],
    [ # frame 4
        [-3, -6], [4, -5.25], [7, -6], [-6, -5.25]
    ],
    [ # frame 5
        [-4, -5], [5, -5.5], [6, -5], [-5, -5.5]
    ]
])

gaits = {
    'stand' : standing_gait,
    'walk' : walking_gait,
    'step' : stepping_gait
}