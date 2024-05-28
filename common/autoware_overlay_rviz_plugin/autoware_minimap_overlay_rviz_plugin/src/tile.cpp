#include "include/tile.hpp"

#include <QDebug>
#include <QImageReader>
#include <QNetworkRequest>

Tile::Tile(int zoom, int x, int y, QObject * parent)
: QObject(parent), zoom_(zoom), x_(x), y_(y), network_manager_(new QNetworkAccessManager(this))
{
  connect(network_manager_, &QNetworkAccessManager::finished, this, &Tile::onTileFetched);
}

Tile::~Tile()
{
}

void Tile::fetch()
{
  requestRemote();
}

bool Tile::isValidUrl(const std::string & url) const
{
  return last_url_ == url;
}

QImage Tile::getImage() const
{
  return image_;
}

std::future<QImage> Tile::requestRemote()
{
  auto const url =
    QUrl(QString("http://tile.openstreetmap.org/%1/%2/%3.png").arg(zoom_).arg(x_).arg(y_));
  QNetworkRequest request(url);
  char constexpr agent[] = "autoware_minimap_overlay_rviz_plugin";
  request.setHeader(QNetworkRequest::UserAgentHeader, agent);

  last_url_ = url.toString().toStdString();  // Save the URL for validation

  std::promise<QImage> tile_promise;
  this->tile_promise_ = std::move(tile_promise);

  // qDebug() << "Fetching tile from URL:" << url;
  network_manager_->get(request);

  return this->tile_promise_.get_future();
}

void Tile::onTileFetched(QNetworkReply * reply)
{
  // qDebug() << "Tile fetch completed.";
  if (reply->error() == QNetworkReply::NoError) {
    // qDebug() << "Reply received successfully.";
    QImageReader reader(reply);
    if (reader.canRead()) {
      // qDebug() << "ImageReader can read the reply.";
      image_ = reader.read();
      if (image_.isNull()) {
        // qDebug() << "Failed to decode the image.";
      } else {
        // qDebug() << "Image successfully fetched. Image size: " <<
        // image_.size();
        tile_promise_.set_value(image_);
        Q_EMIT tileFetched();
      }
    } else {
      // qDebug() << "ImageReader cannot read the reply.";
      tile_promise_.set_value(QImage());
    }
  } else {
    // qDebug() << "Error fetching tile:" << reply->errorString();
    tile_promise_.set_value(QImage());
  }
  reply->deleteLater();
}
