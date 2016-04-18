#! /usr/bin/python
# connect.py: connect to MySQL server

import mysql.connector as mysql
conn_params = {
    'database': 'cookbook',
    'host': "localhost",
    'user': "renye",
    'password': "renye"}


def connect():
    return mysql.connect(**conn_params)
