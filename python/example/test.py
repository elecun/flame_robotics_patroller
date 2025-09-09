import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLineEdit, QPushButton
from PyQt6.QtWebEngineWidgets import QWebEngineView

# HTML 템플릿 (Leaflet.js 이용)
html_template = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Map Viewer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">

  <!-- Leaflet CSS -->
  <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
  <!-- Leaflet JS -->
  <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>

  <style>
    #map { height: 100vh; width: 100%; }
  </style>
</head>
<body>
  <div id="map"></div>
  <script>
    // 초기 지도 설정 (서울)
    var map = L.map('map').setView([37.5665, 126.9780], 13);

    // OSM 타일 로드
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '© OpenStreetMap'
    }).addTo(map);

    var marker = null;

    function setMarker(lat, lng) {
      if (marker) {
        map.removeLayer(marker);
      }
      marker = L.marker([lat, lng]).addTo(map);
      map.setView([lat, lng], 15);
    }
  </script>
</body>
</html>
"""

class MapWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt6 지도 뷰어")

        # 중앙 위젯
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 입력 필드 (위도, 경도)
        self.lat_input = QLineEdit()
        self.lat_input.setPlaceholderText("위도 (예: 37.5665)")
        self.lng_input = QLineEdit()
        self.lng_input.setPlaceholderText("경도 (예: 126.9780)")
        self.button = QPushButton("위치 표시")

        layout.addWidget(self.lat_input)
        layout.addWidget(self.lng_input)
        layout.addWidget(self.button)

        # 웹 뷰 (지도 표시)
        self.view = QWebEngineView()
        self.view.setHtml(html_template)
        layout.addWidget(self.view)

        # 버튼 이벤트
        self.button.clicked.connect(self.show_location)

    def show_location(self):
        try:
            lat = float(self.lat_input.text())
            lng = float(self.lng_input.text())
            js_code = f"setMarker({lat}, {lng});"
            self.view.page().runJavaScript(js_code)
        except ValueError:
            print("위도/경도를 올바르게 입력하세요.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MapWindow()
    window.show()
    sys.exit(app.exec())