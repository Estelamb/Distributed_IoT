# farms/farm_{farm_id}/mission

JSON misión. Node-RED -> Farm.

## Ejemplo

farms/farm_1/mission: {"vehicleId": 1, "commands": [{"missionId": 1, "commandId": 1, "commandType": "TAKEOFF", "params": {"latitude": 40.443546, "longitude": -3.738285}}, {"missionId": 1, "commandId": 2, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443587, "longitude": -3.738355}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443555, "longitude": -3.738452}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443555, "longitude": -3.738452, "treatment": "FERTILIZER", "amount": 0}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443542, "longitude": -3.73856}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443542, "longitude": -3.73856, "treatment": "FERTILIZER", "amount": 2}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443527, "longitude": -3.738693}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443527, "longitude": -3.738693, "treatment": "FERTILIZER", "amount": 1}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443518, "longitude": -3.738819}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443518, "longitude": -3.738819, "treatment": "FERTILIZER", "amount": 1}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443451, "longitude": -3.738816}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443451, "longitude": -3.738816, "treatment": "FERTILIZER", "amount": 2}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443459, "longitude": -3.738682}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443459, "longitude": -3.738682, "treatment": "FERTILIZER", "amount": 1}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443475, "longitude": -3.738546}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443475, "longitude": -3.738546, "treatment": "FERTILIZER", "amount": 0}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443488, "longitude": -3.738429}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443488, "longitude": -3.738429, "treatment": "FERTILIZER", "amount": 0}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443418, "longitude": -3.738421}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443418, "longitude": -3.738421, "treatment": "FERTILIZER", "amount": 1}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443412, "longitude": -3.738528}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443412, "longitude": -3.738528, "treatment": "FERTILIZER", "amount": 2}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443399, "longitude": -3.738673}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443399, "longitude": -3.738673, "treatment": "FERTILIZER", "amount": 2}}, {"missionId": 1, "commandId": 3, "commandType": "GO_WAYPOINT", "params": {"latitude": 40.443381, "longitude": -3.73879}}, {"missionId": 1, "commandId": 4, "commandType": "TREATMENT", "params": {"latitude": 40.443381, "longitude": -3.73879, "treatment": "FERTILIZER", "amount": 2}}, {"missionId": 1, "commandId": 3, "commandType": "GO_HOME", "params": {"latitude": 40.443546, "longitude": -3.738285}}, {"missionId": 1, "commandId": 3, "commandType": "LAND", "params": {"latitude": 40.443546, "longitude": -3.738285}}]}

# farms/farm_{farm_id}/drone_{drone_id}/goal

Confirmación de que el dron ha aceptado la misión y va a comenzar. Farm -> Node-RED.

farms/farm_1/drone_1/goal: {"message": "Goal accepted"}

# farms/farm_{farm_id}/drone_{drone_id}/feedback

Resultado de cada comando. Farm -> Node-RED.

## TAKE OFF

farms/farm_1/drone_1/feedback: {"message": "{\"mission_id\": 1, \"command_id\": 1, \"message\": \"Takeoff successful\", \"latitude\": 40.443546, \"longitude\": -3.738285}"}

## GO WAYPOINT

farms/farm_1/drone_1/feedback: {"message": "{\"mission_id\": 1, \"command_id\": 2, \"message\": \"Waypoint reached\", \"latitude\": 40.443587, \"longitude\": -3.738355}"}

## TREATMENT

farms/farm_1/drone_1/feedback: {"message": "{\"mission_id\": 1, \"command_id\": 4, \"message\": \"Treatment applied: FERTILIZER-0\", \"latitude\": 40.443555, \"longitude\": -3.738452}"}

## GO HOME

farms/farm_1/drone_1/feedback: {"message": "{\"mission_id\": 1, \"command_id\": 3, \"message\": \"Returned home\", \"latitude\": 40.443546, \"longitude\": -3.738285}"}

## LAND

farms/farm_1/drone_1/feedback: {"message": "{\"mission_id\": 1, \"command_id\": 3, \"message\": \"Landing successful\", \"latitude\": 40.443546, \"longitude\": -3.738285}"}

## TAKE PHOTO

farms/farm_1/drone_1/feedback: {"message": "{\"mission_id\": 1, \"command_id\": 4, \"message\": [\"Picture taken with CAM_1\", \"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAIsAAACKCAYAAACaRblaAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAOjTSURBVHheTL13WNZntu7v9TtnT1Q6b++dJjZUlCYdpDdRQLH33rsiChZQsSIW7L333jX2aGyJJaaXSSaZZJKZ2TN79pzP71rf1+xz/niuFxUQ/a53PWvd677v1eLCSycXX3s4+5GLk0+dnHhi48LHLs7da0/13FC6hqto49IR6gzCYw3CYQokzKWnbbiNULcJizEYtdoPnS4As0mD2ajBZNSi16tQawMIUvmhUvsRpGpNQNB7BKtao1b7oNX7ojMG4nTqCA/ 
...
+W4ulym6s1b7HVmsPNprfYaDwMW5motvswd35e7z8DhAVrkyExwmiPBWJ8j1DmK8yMveTOFjuwc3+CD7uing6K2FltBdn68N4OBzH3nI/DlYHcRXfZtN9WOh9haHKZtxtv8fR4it0zm5G7dQmtBQ/xkR1J3F+mqQG6RPrpU6wgwJO2t9hrfYVNhq7MFD4HGf9w3iZn8Tb8hT+NidwN/gWP4u9+ApQ0eEUvpYC/NuLn50CHmaH8LbYRbT7PqxUNmN1/jNkFofxMj5IkLUC/x+apWkewbryXgAAAABJRU5ErkJggg==\"], \"latitude\": 40.443518, \"longitude\": -3.738819}"}

## ANALYSE IMAGE

farms/farm_1/drone_1/feedback: {"message": "{\"mission_id\": 1, \"command_id\": 5, \"message\": [\"Picture analysed: 1\", \"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAIsAAACKCAYAAACaRblaAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAOqVSURBVHheTP11XNVpu/aP+332vmeUWt1dNCqiICndHQooKHZjdwt2ByIWdnd3x+jYMeqMTnfcc8/cvfd+3r/X9VnOfn5/XK+1QEB0HeuM4zzO42pz7qWTC689nH7h4sQTJ8eeWDn3kYczH3Ri9owgYkIVhDm1BDkUeCwKHCYZwS49EaE2gtwmLEYlarU/Op0Ms0mF2ajBZNSg16tQa2UoVH6o1P4oVT7I5G1RKn1Qa3zR6n3RG+U4HVqCgzTEdFXQa7Cdha3RHLiXzvLDQdTPc5NdYSCjyE56gYPkHAsJGXriUnUkZpiJSzURnaQhNkVPUoaZlBw7yZlW0jMslGQ76Jlio3dXA706aajrpGNAjJWBsS7qugaRFmrBqZMRGuShQ/
...
jfa59YS730BW83tGF7YioXS13gYHsNKdT968jswvLQbleNbkNu9EnOlfViq7MVa4wBW6rvRk9uMi8khKVhcHPfi4XMYWdgFwtzOEuF+nkjvi4R6X8DH9Qy25vtxsTuGl6s87o7nMTM4gKPlcdztTmFrdgg7iyM4W5/AxvggZto70FNeh6uV6Im+Q/vSelTOrkNDfhNGytuI9VEnOVDoAqpKojwOmnuwUNmJpdoe9OS24aB7DA/TM3ian8XP8jSuervwNRVl+ADBdmfxMj+Fvc4BfK3lcDM5jKfZXmSuR7BUWo+l4ld4mx/F3Uhsq+X4/wHMUjvbJ78ZbAAAAABJRU5ErkJggg==\"], \"latitude\": 40.443518, \"longitude\": -3.738819}"}

# farms/farm_{farm_id}/drone_{drone_id}/result

Resultado final de la misión. Farm -> Node-RED.

farms/farm_1/drone_1/result: {"message": "Drone 1 mission successful."}

# farms/farm_{farm_id}/warning

Alertas generales de la granja. Farm -> Node-RED.

farms/farm_1/warning: {"message": "Drone 1 server unavailable."}
farms/farm_1/warning: {"message": "Drone 1 is currently busy."}



