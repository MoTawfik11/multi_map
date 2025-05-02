#!/usr/bin/env python3
import sqlite3
import rospkg

def create_database():
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('multi_map_nav')
    db_path = pkg_path + '/wormholes.db'
    
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    
    # Create tables
    c.execute('''CREATE TABLE IF NOT EXISTS wormholes
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  map1 TEXT NOT NULL,
                  map2 TEXT NOT NULL,
                  x1 REAL NOT NULL,
                  y1 REAL NOT NULL,
                  x2 REAL NOT NULL,
                  y2 REAL NOT NULL,
                  radius REAL NOT NULL)''')
    
    conn.commit()
    conn.close()
    print(f"Database created at {db_path}")

if __name__ == '__main__':
    create_database()