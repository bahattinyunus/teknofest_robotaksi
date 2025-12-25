import time
import random
from rich.console import Console
from rich.layout import Layout
from rich.panel import Panel
from rich.live import Live
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, BarColumn, TextColumn
from rich.align import Align
from rich.text import Text

console = Console()

def make_layout() -> Layout:
    layout = Layout(name="root")
    layout.split(
        Layout(name="header", size=3),
        Layout(name="main", ratio=1),
        Layout(name="footer", size=3),
    )
    layout["main"].split_row(
        Layout(name="side"),
        Layout(name="body", ratio=2),
    )
    layout["side"].split(
        Layout(name="system_status"),
        Layout(name="sensors")
    )
    return layout

class RobotaksiDashboard:
    def __init__(self):
        self.layout = make_layout()
        
    def generate_header(self) -> Panel:
        grid = Table.grid(expand=True)
        grid.add_column(justify="center", ratio=1)
        grid.add_column(justify="right")
        grid.add_row(
            "[b]TEKNOFEST ROBOTAKSI COMMAND CENTER[/b] [magenta]v2.0[/magenta]",
            "[i]Cyber-Physical System: ONLINE[/i]",
        )
        return Panel(grid, style="white on blue")

    def generate_system_status(self) -> Panel:
        table = Table(title="System Health", expand=True)
        table.add_column("Module")
        table.add_column("Status")
        
        modules = [
            ("Core", "[green]ACTIVE[/green]"),
            ("Perception", "[green]RUNNING[/green]"),
            ("Planning", "[yellow]CALCULATING[/yellow]"),
            ("Control", "[green]ENGAGED[/green]"),
            ("Safety", "[green]SECURE[/green]")
        ]
        
        for name, status in modules:
            table.add_row(name, status)
            
        return Panel(table, title="[b]Modules[/b]", border_style="cyan")

    def generate_sensor_data(self) -> Panel:
        table = Table(title="Sensor Fusion", expand=True)
        table.add_column("Sensor")
        table.add_column("Data")
        
        lidar_pts = random.randint(15000, 22000)
        fps = round(random.uniform(28.0, 31.0), 1)
        velocity = round(random.uniform(20.0, 45.0), 1)
        
        table.add_row("LiDAR Pts", f"{lidar_pts}")
        table.add_row("Camera FPS", f"{fps}")
        table.add_row("Velocity", f"{velocity} km/h")
        table.add_row("GPS Fix", "3D Fix")
        
        return Panel(table, title="[b]Telemetry[/b]", border_style="magenta")

    def generate_main_view(self) -> Panel:
        # Simulated log output
        logs = [
            "[blue][INFO][/blue] Waypoint 42 reached.",
            "[blue][INFO][/blue] Object detected: Pedestrian (Conf: 0.98)",
            "[yellow][WARN][/yellow] Lane confidence drop on curve.",
            "[blue][INFO][/blue] Global planner updated trajectory.",
            "[green][OK][/green] Heartbeat signal received."
        ]
        
        log_text = "\n".join(logs)
        return Panel(Align.center(f"[b]--- AUTONOMOUS MISSION LOG ---[/b]\n\n{log_text}\n\n[blink]EXECUTING MANEUVER...[/blink]"), title="Mission Monitor", border_style="green")

    def run(self):
        with Live(self.layout, refresh_per_second=4, screen=True):
            while True:
                self.layout["header"].update(self.generate_header())
                self.layout["system_status"].update(self.generate_system_status())
                self.layout["sensors"].update(self.generate_sensor_data())
                self.layout["body"].update(self.generate_main_view())
                time.sleep(0.2)

if __name__ == "__main__":
    try:
        dashboard = RobotaksiDashboard()
        dashboard.run()
    except KeyboardInterrupt:
        print("System Shutdown.")
