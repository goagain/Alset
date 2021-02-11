from PyQt5.QtCore import QObject, QTimer
import weakref


class Tickable():
    def __init__(self) -> None:
        TickerManager.instance().add(self)

    def on_tick(self):
        pass


class TickerManager(QObject):
    _instance = None

    def init(parent: QObject = None):
        TickerManager._instance = TickerManager(parent)

    def instance():
        return TickerManager._instance

    def __init__(
        self,
        parent: QObject = None,
    ) -> None:
        super().__init__(parent=parent)
        self.ticker = QTimer(self)
        self.ticker.start(20)
        self.ticker.timeout.connect(self.tick)

        self.objects = weakref.WeakSet()

    def add(self, obj):
        self.objects.add(obj)

    def tick(self):
        for obj in self.objects:
            if obj:
                obj.on_tick()