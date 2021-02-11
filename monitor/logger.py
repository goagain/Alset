def log(func):
    def inner(*args, **kwargs):
        print(f'calling {func.__name__} with args={args} and kwargs={kwargs}')
        return func(*args, **kwargs)
    return inner