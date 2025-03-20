class TableMeta(type):
    def __repr__(cls):
        return f"{cls.__name__}({cls._keys()})"

    def _dict(cls):
        return cls.__annotations__

    def _keys(cls):
        keys = list(cls._dict().keys())
        return ", ".join(str(key) for key in keys)

    def _columns(cls):
        return ", ".join(
            f"{key} {cls._sqlite_type(value)}" for key, value in cls._dict().items()
        )

    def _sqlite_type(cls, value):
        if value is int:
            return "INTEGER"
        elif value is float:
            return "REAL"
        elif value is str:
            return "TEXT"
        elif value is bytes:
            return "BLOB"
        else:
            return "TEXT"


class Table(metaclass=TableMeta):
    pass

    def __repr__(self):
        attrs = ", ".join(
            f"{key}={getattr(self, key)!r}"
            for key in self.__class__._keys().split(", ")
        )
        return f"{self.__class__.__name__}({attrs})"
