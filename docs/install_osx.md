# Installation Instructions for OSX

1. Install [pyenv](https://github.com/pyenv/pyenv)
2. Install [pipenv](https://pypi.org/project/pipenv/)
3. Run `pipenv install` to install the correct version of python and the pip packages
4. Go to the `space_invaders/db` directory and create the database game_logs.db:
    ```bash
    $ cd space_invaders/db
    $ sqlite3 game_logs.db < create_database.sql 
    ```
5. Return to main directory:
   ```bash
   $ cd ../..
   ```
6. You may now test the game by executing `pipenv run python space_invaders/websocket.py` and going to the link it provides in the terminal (you must be within the activated virual environment when you do this)
