from Patient import Patient
from SQLite import db


def main():
    print("Hi from db.")
    me = Patient("Ujaan")
    print(Patient)
    print(me)
    db.create_table(Patient)
    db.insert_row(me)
    db.fetch_rows(Patient)


if __name__ == "__main__":
    main()
