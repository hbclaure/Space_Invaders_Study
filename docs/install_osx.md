# Installation Instructions for OSX

1. Download Python 3.7.3 from [here](https://www.python.org/downloads/) and install it in your compute
2. Go to the `space_invaders/db` directory and create the database game_logs.db:
    ```bash
    $ cd space_invaders_web/space_invaders/db
    $ sqlite3 game_logs.db < create_database.sql 
    ```
3. Go to the `space_invaders` directory and create a virtual environment:
    ```bash
    $ cd space_invaders_web/space_invaders
    $ python3 -m venv venv
    ```
4. Activate the environment by running `. venv/bin/activate`
5. Within the activated environment, run `pip install Flask` to install flask
6. You may now test the game by executing `flask run` and going to the link it provides in the terminal (you must be within the activated virual environment when you do this)