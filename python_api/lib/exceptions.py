class GetGlobalVariableError(Exception):
    def __init__(self, key, file, message: str = None):
        self.message = (
            f'The {key} variable is missing from the global variables '
            f'file {file}. '
        )
        if message:
            self.message = self.message + message
        super().__init__(self.message)
