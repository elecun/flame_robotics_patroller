import time

import numpy as np

import viser


def main() -> None:
    server = viser.ViserServer()

    server.scene.add_frame("/axes", axes_length=1.0, axes_radius=0.02)

    # A panel docked to the right edge, with two tabs. Panels start
    # expanded; minimize() below collapses the log panel imperatively, and
    # the user can minimize/expand freely in the browser.
    stats_panel = server.gui.add_panel()
    with stats_panel.add_tab("Stats", viser.Icon.CHART_BAR):
        counter = server.gui.add_number("Counter", initial_value=0, disabled=True)
        server.gui.add_markdown("Live values update in this docked panel.")
    with stats_panel.add_tab("Notes", viser.Icon.NOTES):
        server.gui.add_markdown("A second tab in the same panel.")
    stats_panel.dock_right()
    stats_panel.set_width(320)

    # A floating panel at an explicit position. x/y are viewport-relative (the
    # canvas inside docked panels), so this stays clear of the left-docked main
    # panel below and shifts if the docked regions change.
    tools_panel = server.gui.add_panel()
    with tools_panel.add_tab("Tools", viser.Icon.TOOL):
        randomize = server.gui.add_button("Randomize point cloud")
    tools_panel.float(x=30, y=30, width=260)
    # Start the tools panel minimized (a floating panel collapses to its
    # header bar). Collapse applies to the panel's CONTAINER -- panels
    # stacked together minimize together -- so we demo it on a panel with
    # its own window; log_panel below shares a column with stats_panel,
    # and minimizing it would rail them both.
    tools_panel.minimize()

    # A panel stacked below the docked stats panel (a column split).
    log_panel = server.gui.add_panel()
    with log_panel.add_tab("Log", viser.Icon.TERMINAL):
        log = server.gui.add_markdown("Waiting for events...")
    log_panel.dock_below(stats_panel)

    # The main control panel can be placed too -- here, docked to the left.
    server.gui.main_panel.dock_left()

    rng = np.random.default_rng(0)
    points = rng.normal(size=(2000, 3)) * 0.5
    log_lines: list[str] = []

    def append_log(message: str) -> None:
        # Keep a short rolling history so the log reads like a real log, not a
        # single replaced line.
        log_lines.append(message)
        del log_lines[:-8]  # keep the last 8 lines
        log.content = "\n\n".join(log_lines)

    @randomize.on_click
    def _(_) -> None:
        nonlocal points
        points = rng.normal(size=(2000, 3)) * 0.5
        append_log(f"Randomized at t={counter.value}.")

    while True:
        counter.value += 1
        server.scene.add_point_cloud(
            "/points", points=points, colors=(120, 180, 255), point_size=0.02
        )
        time.sleep(0.5)


if __name__ == "__main__":
    main()