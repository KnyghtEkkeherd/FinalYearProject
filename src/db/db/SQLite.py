import sqlite3
from Table import Table


class SQLite:
    def __init__(self):
        self.con = sqlite3.connect("fyp.db")
        self.cur = self.con.cursor()

    def _is_table(self, table_cls: type):
        if not issubclass(table_cls, Table):
            raise ValueError("The provided class must inherit from Table.")

    def create_table(self, table_cls: type):
        self._is_table(table_cls)

        table_name = table_cls.__name__
        columns = table_cls._columns()
        self.cur.execute(f"CREATE TABLE IF NOT EXISTS {table_name} ({columns})")
        print(f"Table '{table_name}' created with columns: {columns}")

    def insert_row(self, instance: Table):
        self._is_table(type(instance))

        table_name = type(instance).__name__
        keys = ", ".join(instance.__class__._keys().split(", "))
        placeholders = ", ".join("?" for _ in instance.__class__._dict().keys())
        values = tuple(
            getattr(instance, key) for key in instance.__class__._dict().keys()
        )
        self.cur.execute(
            f"INSERT INTO {table_name} ({keys}) VALUES ({placeholders})", values
        )
        self.con.commit()
        print(f"Row inserted into table '{table_name}': {instance}")

    def fetch_rows(self, table_cls: type):
        self._is_table(table_cls)

        table_name = table_cls.__name__
        self.cur.execute(f"SELECT * FROM {table_name}")
        print(self.cur.fetchall())

    def close(self):
        self.con.close()


db = SQLite()
