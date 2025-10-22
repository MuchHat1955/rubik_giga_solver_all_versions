/*

{
  "main": {
    "title": "rubik cube solver",
    "footer": "ready",
    "columns": 1,
    "rows": [
      [{ "text": "solve cube", "type": "menu", "key": "solve" }],
      [{ "text": "read cube", "type": "menu", "key": "read" }],
      [{ "text": "scramble cube", "type": "menu", "key": "random" }],
      [{ "text": "tests", "type": "menu", "key": "tests" }],
      [{ "text": "system", "type": "menu", "key": "system" }]
    ]
  },
  "solve": {
    "title": "solve cube",
    "footer": "press to start solving",
    "columns": 1,
    "rows": [
      [{ "text": "start", "type": "action", "key": "run_solve" }],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "read": {
    "title": "read cube",
    "footer": "read cube colors",
    "columns": 1,
    "rows": [
      [{ "text": "start read", "type": "action", "key": "run_read" }],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "random": {
    "title": "scramble cube",
    "footer": "scramble cube randomly",
    "columns": 1,
    "rows": [
      [{ "text": "scramble (12)", "type": "action", "key": "scramble_12" }],
      [{ "text": "scramble (20)", "type": "action", "key": "scramble_20" }],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "system": {
    "title": "system status",
    "footer": "scroll for details",
    "columns": 1,
    "equal columns": "all",
    "rows": [
      [{ "type": "error_status", "text": "" }],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "tests": {
    "title": "tests",
    "footer": "perform hardware tests",
    "columns": 2,
    "equal columns": "all",
    "rows": [
      [
        { "text": "servo ids", "type": "menu", "key": "servos_programming" },
        { "text": "servo limits", "type": "menu", "key": "servo_limits" }
      ],
      [
        { "text": "vertical tune", "type": "menu", "key": "vertical_tune" },
        { "text": "vertical poses", "type": "menu", "key": "vertical_poses" }
      ],
      [
        { "text": "poses", "type": "menu", "key": "poses" },
        { "text": "pose groups", "type": "menu", "key": "pose_groups" }
      ],
      [
        { "text": "sequences", "type": "menu", "key": "sequences" },
        { "text": "cube moves", "type": "menu", "key": "cube_moves" }
      ],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "servo_limits": {
    "title": "servo limits",
    "footer": "touch to edit numeric limits",
    "columns": 4,
    "equal columns": "last",
    "rows": [
      [
        { "text": "servo", "type": "text" },
        { "text": "0 degree", "type": "text" },
        { "text": "min limit", "type": "text" },
        { "text": "max limit", "type": "text" }
      ],
      [
        { "text": "arm1", "type": "action", "status": "yes", "key": "arm1" },
        { "text": " -0000+ ", "type": "num", "key": "arm1_0" },
        { "text": " -0000+ ", "type": "num", "key": "arm1_min" },
        { "text": " -0000 +", "type": "num", "key": "arm1_max" }
      ],
      [
        { "text": "arm2", "type": "action", "status": "yes", "key": "arm2" },
        { "text": "-0000+", "type": "num", "key": "arm2_0" },
        { "text": "-0000+", "type": "num", "key": "arm2_min" },
        { "text": "-0000+", "type": "num", "key": "arm2_max" }
      ],
      [
        { "text": "wrist", "type": "action", "status": "yes", "key": "wrist" },
        { "text": "-0000+", "type": "num", "key": "wrist_0" },
        { "text": "-0000+", "type": "num", "key": "wrist_min" },
        { "text": "-0000+", "type": "num", "key": "wrist_max" }
      ],
      [
        { "text": "grip l", "type": "action", "status": "yes", "key": "grip1" },
        { "text": "-0000+", "type": "num", "key": "grip1_0" },
        { "text": "-0000+", "type": "num", "key": "grip1_min" },
        { "text": "-0000+", "type": "num", "key": "grip1_max" }
      ],
      [
        { "text": "grip r", "type": "action", "status": "yes", "key": "grip2" },
        { "text": "-0000+", "type": "num", "key": "grip2_0" },
        { "text": "-0000+", "type": "num", "key": "grip2_min" },
        { "text": "-0000+", "type": "num", "key": "grip2_max" }
      ],
      [
        { "text": "base", "type": "action", "status": "yes", "key": "base" },
        { "text": "-0000+", "type": "num", "key": "base_0" },
        { "text": "-0000+", "type": "num", "key": "base_min" },
        { "text": "-0000+", "type": "num", "key": "base_max" }
      ],
      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  },
  "servos_programming": {
    "title": "tests",
    "footer": "set servo id and reset to mid range",
    "columns": 4,
    "equal columns": "last",
    "rows": [
      [
        { "text": "arm1 (id11)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "arm1_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "arm1_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "arm1_status" }
      ],
      [
        { "text": "arm2 (id12)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "arm2_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "arm2_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "arm2_status" }
      ],
      [
        { "text": "wrist (id13)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "wrist_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "wrist_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "wrist_status" }
      ],
      [
        { "text": "grip l (id14)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "grip1_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "grip1_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "grip1_status" }
      ],
      [
        { "text": "grip r (id15)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "grip2_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "grip2_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "grip2_status" }
      ],
      [
        { "text": "base (id16)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "base_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "base_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "base_status" }
      ],
      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  },
  "pose_groups": {
  "title": "pose groups",
  "footer": "tap to run pose groups",
  "columns": 3,
  "equal columns": "all",
  "rows": [
    [
      { "text": "arms home", "type": "action", "key": "arms_home", "status": "yes" },
      { "text": "arms 2nd",  "type": "action", "key": "arms_2nd",  "status": "yes" },
      { "text": "arms 3rd",  "type": "action", "key": "arms_3rd",  "status": "yes" }
    ],
    [
      { "text": "arms r1", "type": "action", "key": "arms_r1", "status": "yes" },
      { "text": "arms r2", "type": "action", "key": "arms_r2", "status": "yes" },
      { "text": "arms r3", "type": "action", "key": "arms_r3", "status": "yes" }
    ],    
    [
      { "text": "arms r4", "type": "action", "key": "arms_r4", "status": "yes" },
      { "text": "arms r5", "type": "action", "key": "arms_r5", "status": "yes" },
      { "text": "arms r6", "type": "action", "key": "arms_r6", "status": "yes" }
    ],
    [
      { "text": "grippers open", "type": "action", "key": "grip_open", "status": "yes" },
      { "text": "grippers closed", "type": "action", "key": "grip_closed", "status": "yes" },
      { "text": "", "type": "text", "key": "" }
    ],
    [
      { "text": "back", "type": "menu", "key": "tests" }
    ]
  ]
  },
  "sequences": {
  "title": "sequences",
  "footer": "tap to execute motion sequence",
  "columns": 2,
  "equal columns": "all",
  "rows": [
    [
      { "text": "bottom+", "type": "action", "key": "bottom_plus",  "status": "yes" },
      { "text": "bottom-", "type": "action", "key": "bottom_minus", "status": "yes" }
    ],
    [
      { "text": "front to base", "type": "action", "key": "front_to_base", "status": "yes" },
      { "text": "back to base",  "type": "action", "key": "back_to_base",  "status": "yes" }
    ],
    [
      { "text": "left to base",  "type": "action", "key": "left_to_base",  "status": "yes" },
      { "text": "right to base", "type": "action", "key": "right_to_base", "status": "yes" }
    ],
    [
      { "text": "top to base",   "type": "action", "key": "top_to_base",   "status": "yes" },
      { "text": "", "type": "text", "key": "blank" }
    ],
    [
      { "text": "rotate down face+", "type": "action", "key": "rotate_down_90",      "status": "yes" },
      { "text": "rotate down face-", "type": "action", "key": "rotate_down_minus90", "status": "yes" }
    ],
    [
      { "text": "back", "type": "menu", "key": "tests" }
    ]
  ]
},
"cube_moves": {
  "title": "cube moves",
  "footer": "tap to run a move",
  "columns": 4,
  "equal columns": "all",
  "rows": [
    [
      { "text": "f+", "type": "action", "key": "f_plus",  "status": "yes" },
      { "text": "f-", "type": "action", "key": "f_minus", "status": "yes" },
      { "text": "b+", "type": "action", "key": "b_plus",  "status": "yes" },
      { "text": "b-", "type": "action", "key": "b_minus", "status": "yes" }
    ],
    [
      { "text": "u+", "type": "action", "key": "u_plus",  "status": "yes" },
      { "text": "u-", "type": "action", "key": "u_minus", "status": "yes" },
      { "text": "d+", "type": "action", "key": "d_plus",  "status": "yes" },
      { "text": "d-", "type": "action", "key": "d_minus", "status": "yes" }
    ],
    [
      { "text": "l+", "type": "action", "key": "l_plus",  "status": "yes" },
      { "text": "l-", "type": "action", "key": "l_minus", "status": "yes" },
      { "text": "r+", "type": "action", "key": "r_plus",  "status": "yes" },
      { "text": "r-", "type": "action", "key": "r_minus", "status": "yes" }
    ],
    [
      { "text": "f++", "type": "action", "key": "f_plusplus", "status": "yes" },
      { "text": "b++", "type": "action", "key": "b_plusplus", "status": "yes" },
      { "text": "u++", "type": "action", "key": "u_plusplus", "status": "yes" },
      { "text": "d++", "type": "action", "key": "d_plusplus", "status": "yes" }
    ],
    [
      { "text": "l++", "type": "action", "key": "l_plusplus", "status": "yes" },
      { "text": "r++", "type": "action", "key": "r_plusplus", "status": "yes" },
      { "text": "", "type": "text", "key": "blank" },
      { "text": "", "type": "text", "key": "blank" }
    ],
    [
      { "text": "back", "type": "menu", "key": "tests" }
    ]
  ]
},
  "vertical_tune": {
  "title": "vertical tune",
  "footer": "adjust for vertical move arm1, arm2, wrist",
  "columns": 5,
  "rows": [
    [
      { "text": "set", "type": "text", "key": "" },
      { "text": "clr", "type": "text", "key": "" },
      { "text": "arm1", "type": "text" },
      { "text": "arm2", "type": "text" },
      { "text": "wrist", "type": "text" }
    ],
    [
      { "text": "s1", "type": "action", "key": "vertical_p1_set", "status": "yes" },
      { "text": "c1", "type": "action", "key": "vertical_p1_clr", "status": "yes" },
      { "text": "value", "type": "num", "key": "arm1_v_pt1" },
      { "text": "value", "type": "num", "key": "arm2_v_pt1" },
      { "text": "value", "type": "num", "key": "wrist_v_pt1" }
    ],
      [
      { "text": "s2", "type": "action", "key": "vertical_p2_set", "status": "yes" },
      { "text": "c2", "type": "action", "key": "vertical_p2_clr", "status": "yes" },
      { "text": "value", "type": "num", "key": "arm1_v_pt2" },
      { "text": "value", "type": "num", "key": "arm2_v_pt2" },
      { "text": "value", "type": "num", "key": "wrist_v_pt1" }
    ],
    [
      { "text": "s3", "type": "action", "key": "vertical_p3_set", "status": "yes" },
      { "text": "c3", "type": "action", "key": "vertical_p3_clr", "status": "yes" },
      { "text": "value", "type": "num", "key": "arm1_v_pt3" },
      { "text": "value", "type": "num", "key": "arm3_v_pt3" },
      { "text": "value", "type": "num", "key": "wrist_v_pt1" }
    ],
    [
      { "text": "s4", "type": "action", "key": "vertical_p4_set", "status": "yes" },
      { "text": "c4", "type": "action", "key": "vertical_p4_clr", "status": "yes" },
      { "text": "value", "type": "num", "key": "arm1_v_pt4" },
      { "text": "value", "type": "num", "key": "arm2_v_pt4" },
      { "text": "value", "type": "num", "key": "wrist_v_pt1" }
    ],
    [
      { "text": "s5", "type": "action", "key": "vertical_p5_set", "status": "yes" },
      { "text": "c5", "type": "action", "key": "vertical_p5_clr", "status": "yes" },
      { "text": "value", "type": "num", "key": "arm1_v_pt5" },
      { "text": "value", "type": "num", "key": "arm2_v_pt5" },
      { "text": "value", "type": "num", "key": "wrist_v_pt5" }
    ],
    [
      { "text": "s6", "type": "action", "key": "vertical_p6_set", "status": "yes" },
      { "text": "c6", "type": "action", "key": "vertical_p6_clr", "status": "yes" },
      { "text": "value", "type": "num", "key": "arm1_v_pt6" },
      { "text": "value", "type": "num", "key": "arm2_v_pt6" },
      { "text": "value", "type": "num", "key": "wrist_v_pt6" }
    ],
    [
      { "text": "g1 open", "type": "action", "key": "grip1_open", "status": "yes" },
    ],
    [
      { "text": "g1 close", "type": "action", "key": "grip1_close", "status": "yes" },
    ],
    [
      { "text": "g2 open", "type": "action", "key": "grip2_open", "status": "yes" },
    ],
    [
      { "text": "g2 close", "type": "action", "key": "grip2_close", "status": "yes" },
    ],
    [
      { "text": "back", "type": "menu", "key": "tests" }
    ]
  ]
  },
  "vertical_poses": {
  "title": "vertical poses",
  "footer": "test vertical poses",
  "columns": 3,
  "equal columns": "last",
  "rows": [
    [
      { "text": "pose", "type": "text" },
      { "text": "x mm", "type": "text" },
      { "text": "y mm", "type": "text" }
    ],
    [
      { "text": "arms home", "type": "action", "key": "v_pose_0", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_0_x" },
      { "text": "value", "type": "num", "key": "v_pose_0_y" }
    ],
    [
      { "text": "arms 2nd", "type": "action", "key": "v_pose_2nd", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_2nd_x" },
      { "text": "value", "type": "num", "key": "v_pose_2nd_y" }
    ],
    [
      { "text": "arms 3rd", "type": "action", "key": "v_pose_3rd", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_3rd_x" },
      { "text": "value", "type": "num", "key": "v_pose_3rd_y" }
    ],
     [
      { "text": "arms r1", "type": "action", "key": "v_pose_r1", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_r1_x" },
      { "text": "value", "type": "num", "key": "v_pose_r1_y" }
    ],   
    [
      { "text": "arms r2", "type": "action", "key": "v_pose_r2", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_r2_x" },
      { "text": "value", "type": "num", "key": "v_pose_r2_y" }
    ],
    [
      { "text": "arms r3", "type": "action", "key": "v_pose_r3", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_r3_x" },
      { "text": "value", "type": "num", "key": "v_pose_r3_y" }
    ],   
     [
      { "text": "arms r1", "type": "action", "key": "v_pose_r4", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_r4_x" },
      { "text": "value", "type": "num", "key": "v_pose_r4_y" }
    ],   
    [
      { "text": "arms r2", "type": "action", "key": "v_pose_r5", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_r5_x" },
      { "text": "value", "type": "num", "key": "v_pose_r5_y" }
    ],
    [
      { "text": "arms r3", "type": "action", "key": "v_pose_r6", "status": "yes" },
      { "text": "value", "type": "num", "key": "v_pose_r6_x" },
      { "text": "value", "type": "num", "key": "v_pose_r6_y" }
    ], 
    [
      { "text": "back", "type": "menu", "key": "tests" }
    ]
   ]
  },
  "poses": {
    "title": "servo poses",
    "footer": "tap to edit pose values",
    "columns": 3,
    "equal columns": "all",
    "rows": [
      [{ "text": "arm1 0", "type": "action", "key": "arm1_0", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "arm1_0" },
       { "text": "current na", "type": "text", "key": "arm1_current" }],

      [{ "text": "arm2 0", "type": "action", "key": "arm2_0", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "arm2_0" },
       { "text": "current na", "type": "text", "key": "arm2_current" }],

      [{ "text": "wrist 0", "type": "action", "key": "wrist_0", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "wrist_0" },
       { "text": "current na", "type": "text", "key": "wrist_current" }],

      [{ "text": "wrist 90", "type": "action", "key": "wrist_90", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "wrist_90" },
       { "text": "current na", "type": "text", "key": "wrist_current" }],

      [{ "text": "wrist -90", "type": "action", "key": "wrist_minus90", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "wrist_minus90" },
       { "text": "current na", "type": "text", "key": "wrist_current" }],

      [{ "text": "grip1 open", "type": "action", "key": "grip1_open", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "grip1_open" },
       { "text": "current na", "type": "text", "key": "grip1_current" }],

      [{ "text": "grip1 close", "type": "action", "key": "grip1_close", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "grip1_close" },
       { "text": "current na", "type": "text", "key": "grip1_current" }],

      [{ "text": "grip2 open", "type": "action", "key": "grip2_open", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "grip2_open" },
       { "text": "current na", "type": "text", "key": "grip2_current" }],

      [{ "text": "grip2 close", "type": "action", "key": "grip2_close", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "grip2_close" },
       { "text": "current na", "type": "text", "key": "grip2_current" }],

      [{ "text": "base 0", "type": "action", "key": "base_0", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "base_0" },
       { "text": "current na", "type": "text", "key": "base_current" }],

      [{ "text": "base 90", "type": "action", "key": "base_90", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "base_90" },
       { "text": "current na", "type": "text", "key": "base_current" }],

      [{ "text": "base -90", "type": "action", "key": "base_minus90", "status": "yes" },
       { "text": " +0000- ", "type": "num", "key": "base_minus90" },
       { "text": "current na", "type": "text", "key": "base_current" }],

      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  }
}

*/