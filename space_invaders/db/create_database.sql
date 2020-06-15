BEGIN TRANSACTION;
CREATE TABLE IF NOT EXISTS "Events" (
	"id"	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
	"frame_id"	INTEGER,
	"killer"	TEXT,
	"killed"	TEXT,
	FOREIGN KEY("frame_id") REFERENCES "Frames"("id")
);
CREATE TABLE IF NOT EXISTS "Games" (
	"id"	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
	"player_id"	TEXT,
	"date"	TEXT,
	"mode"	INTEGER,
	"round"	INTEGER
);
CREATE TABLE IF NOT EXISTS "Frames" (
	"id"	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
	"game_id"	INTEGER NOT NULL,
	"frame_number"	INTEGER,
	"player_position"	INTEGER,
	"player_lives"	INTEGER,
	"player_score"	INTEGER,
	"ai_position"	INTEGER,
	"ai_lives"	INTEGER,
	"ai_score"	INTEGER,
	FOREIGN KEY("game_id") REFERENCES "Games"("id")
);
CREATE TABLE IF NOT EXISTS "Bullets" (
	"id"	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
	"frame_id"	INTEGER NOT NULL,
	"type"	INTEGER,
	"x"	INTEGER,
	"y"	INTEGER,
	FOREIGN KEY("frame_id") REFERENCES "Frames"("id")
);
CREATE TABLE IF NOT EXISTS "Enemies" (
	"id"	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
	"frame_id"	INTEGER NOT NULL,
	"side"	TEXT,
	"x"	INTEGER,
	"y"	INTEGER,
	FOREIGN KEY("frame_id") REFERENCES "Frames"("id")
);
COMMIT;
