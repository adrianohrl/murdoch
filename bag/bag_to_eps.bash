BAG_FILE="analytics.bag"

TOPIC="/analytics/cycles"
CSV_FILE="cycles.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/waypoints"
CSV_FILE="waypoints.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/waypoints/x"
CSV_FILE="waypoints-x.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/waypoints/y"
CSV_FILE="waypoints-y.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/skills/camera"
CSV_FILE="skills-camera.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/skills/battery"
CSV_FILE="skills-battery.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/skills/battery/level"
CSV_FILE="skills-level-battery.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/skills/strength"
CSV_FILE="skills-strength.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/skills/strength/level"
CSV_FILE="skills-level-strength.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/skills/processor"
CSV_FILE="skills-processor.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/skills/processor/level"
CSV_FILE="skills-level-processor.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

TOPIC="/analytics/skills/laserscan"
CSV_FILE="skills-laserscan.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} > ${CSV_FILE}

OCTAVE_EXPRESSION="bag_to_eps"
octave --silent --eval "${OCTAVE_EXPRESSION}"


