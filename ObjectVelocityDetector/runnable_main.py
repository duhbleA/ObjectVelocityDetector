import main
from constants import constants

main.start()
while True:
    main.execute_session("0")
    
    if constants.NUMBER_OF_CAMERAS > 1:
        main.execute_session("1")
        
    if main.window_terminated_requested():
        break

