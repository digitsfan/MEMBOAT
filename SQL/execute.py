#! /usr/bin/python
# connect.py: connect to MySQL server
import cookbook

# open connection
conn = cookbook.connect()
print("Connected")

# insert cursor
cursor = conn.cursor()
# cursor.execute("UPDATE profile SET cats = cats+1 WHERE name = 'Sybil';")
# print("Number of rows updated: %d" % cursor.rowcount)

# query cursor
cursor.execute('SELECT id, name, cats FROM profile;')
# # alternative 1
# while True:
#     row = cursor.fetchone()
#     if row is None:
#         break
#     print("id: %s, name: %s, cats: %s" % (row[0], row[1], row[2]))
# print("Number of rows returned: %d" % cursor.rowcount)

# # alternative 2
# for (id, name, cats) in cursor:
#     print("id: %s, name: %s, cats: %s" % (id, name, cats))
# print("Number of rows returned: %d" % cursor.rowcount)

# alternative 3
rows = cursor.fetchall()
for row in rows:
    print("id: %s, name: %s, cats: %s" % (row[0], row[1], row[2]))
print("Number of rows returned: %d" % cursor.rowcount)

cursor.close()

# close connection
conn.commit()
conn.close()
print("Disconnected")
