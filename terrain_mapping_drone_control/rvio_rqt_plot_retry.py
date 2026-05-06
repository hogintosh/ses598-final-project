#!/usr/bin/env python3

import sys

from python_qt_binding.QtCore import QTimer
from rqt_gui.main import Main
from rqt_plot import plot_widget


ORIGINAL_INIT = plot_widget.PlotWidget.__init__


def retry_initial_topics(self):
    if not self._rvio_pending_initial_topics:
        self._rvio_retry_timer.stop()
        return

    for topic_name in list(self._rvio_pending_initial_topics):
        self.add_topic(topic_name)
        if topic_name in self._rosdata:
            self._rvio_pending_initial_topics.remove(topic_name)

    self._rvio_retry_attempts += 1
    if not self._rvio_pending_initial_topics:
        self._rvio_retry_timer.stop()
    elif self._rvio_retry_attempts >= 60:
        print(
            'rvio_rqt_plot_retry: failed to subscribe to %s'
            % ', '.join(self._rvio_pending_initial_topics),
            file=sys.stderr,
        )
        self._rvio_retry_timer.stop()


def retrying_init(self, node, initial_topics=None, start_paused=False):
    self._rvio_pending_initial_topics = list(initial_topics or [])
    self._rvio_retry_attempts = 0
    ORIGINAL_INIT(self, node, initial_topics=[], start_paused=start_paused)

    if self._rvio_pending_initial_topics:
        self._rvio_retry_timer = QTimer(self)
        self._rvio_retry_timer.timeout.connect(lambda: retry_initial_topics(self))
        self._rvio_retry_timer.start(500)
        QTimer.singleShot(100, lambda: retry_initial_topics(self))


def main():
    plot_widget.PlotWidget.__init__ = retrying_init

    import rqt_plot.plot

    plugin = 'rqt_plot.plot.Plot'
    rqt_main = Main(filename=plugin)
    sys.exit(
        rqt_main.main(
            standalone=plugin,
            plugin_argument_provider=rqt_plot.plot.Plot.add_arguments,
        )
    )


if __name__ == '__main__':
    main()
