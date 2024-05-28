#ifndef TILE_HPP
#define TILE_HPP

#include <QImage>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QObject>
#include <future>

class Tile : public QObject {
  Q_OBJECT

public:
  Tile(int zoom, int x, int y, QObject *parent = nullptr);
  ~Tile();
  void fetch();
  QImage getImage() const;
  bool isValidUrl(const std::string &url) const;

  int getZoom() const { return zoom_; }
  int getX() const { return x_; }
  int getY() const { return y_; }

Q_SIGNALS:
  void tileFetched();

private Q_SLOTS:
  void onTileFetched(QNetworkReply *reply);

private:
  int zoom_;
  int x_;
  int y_;
  std::string last_url_;
  QImage image_;
  QNetworkAccessManager *network_manager_;

  std::promise<QImage> tile_promise_;

  std::future<QImage> requestRemote();
};

#endif // TILE_HPP