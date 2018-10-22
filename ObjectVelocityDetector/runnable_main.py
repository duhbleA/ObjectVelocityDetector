import main

main.start()
while True:
    main.execute_session("0")
    main.execute_session("1")
    if main.window_terminated_requested():
        break

