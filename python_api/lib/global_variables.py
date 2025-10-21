import json

from lib.exceptions import GetGlobalVariableError


class GlobalVariablesManager:
    """
    Управление глобальными переменными, сохраняемыми в JSON файл.

    Args:
        file_path (str): Путь к файлу JSON, в котором сохраняются и извлекаются
            глобальные переменные.

    Attributes:
        file_path (str): Путь к файлу JSON.
        data (dict): Словарь, содержащий загруженные глобальные переменные из
            файла.
    """
    # TODO: написать документацию

    def __init__(self, file_path='global_variables.json'):
        self.file_path = file_path
        self.data = self._load()

    def _load(self):
        """
        Загружает глобальные переменные из файла JSON при инициализации
        экземпляра класса.

        Returns:
            dict: Словарь с загруженными глобальными переменными. Если файл не
                найден или не может быть прочитан, возвращается пустой словарь.
        """

        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            with open(self.file_path, 'w', encoding='utf-8') as f:
                self._create_empty_file()

    def _create_empty_file(self):
        """
        Создает пустой JSON файл.
        """
        with open(self.file_path, 'w', encoding='utf-8') as f:
            json.dump({}, f)

    def _save(self):
        """
        Сохраняет текущее состояние глобальных переменных в файл JSON.
        """

        with open(self.file_path, 'w', encoding='utf-8') as f:
            json.dump(self.data, f, indent=4)

    def set(self, key, value):
        """
        Устанавливает значение для указанного ключа в словаре глобальных
        переменных и сохраняет их в файл.

        Args:
            key (str): Ключ для установки значения.
            value: Значение, которое нужно установить для указанного ключа.
        """

        self.data[key] = value
        self._save()

    def get(self, key):
        """
        Возвращает значение для указанного ключа из словаря глобальных
        переменных.

        Args:
            key (str): Ключ, значение которого нужно получить.

        Returns:
            Значение, соответствующее указанному ключу.

        Raises:
            GetGlobalVariableError: Если ключ отсутствует в файле глобальных
                переменных.
        """
        try:
            return self.data[key]
        except KeyError:
            raise GetGlobalVariableError(key=key, file=self.file_path)
