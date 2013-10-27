import sqlite3


if __name__ == '__main__':
	
	conn = sqlite3.connect('results.sqlite')
	c = conn.cursor()
	try:
		c.execute('''CREATE TABLE problem_setups
		(id INTEGER PRIMARY KEY AUTOINCREMENT, starts text,goals text,map_name text)''')
	except sqlite3.OperationalError as e:
		print e
	try:
		c.execute('''CREATE TABLE runs
		(id INTEGER PRIMARY KEY AUTOINCREMENT,problem_id integer, num_drones integer, mpm text,tdm text, plan_time real, plan_success boolean,plan_execution_time real,total_travel_distance real,map_name text,
		FOREIGN KEY(problem_id) REFERENCES problem_setups(id))''')
	except sqlite3.OperationalError as e:
		print e
	conn.commit()
	c.close()