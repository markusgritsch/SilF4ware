#! python2.5

import os, stat, sys

from qt import *

drawVersion = True
try:
    import versioninfo
except ImportError:
    drawVersion = False

tabWidth = 4
lineHeight = 3
outputFolder = '_birdseyeviews'

columnWidth = 128

pixmapMap = {}
def render( filepath ):
    if os.path.isfile( filepath ) and ( filepath.endswith( '.c' ) or filepath.endswith( '.h' ) ) \
      and not __file__.endswith( filepath ) and not "stm32f4xx" in filepath and not "main.c" == filepath:
        ##pixmapFilepath = os.path.join( outputFolder, os.path.basename( filepath.replace( os.sep, '_' ).replace( ':', '_' ) ) + '.png' )
        pixmapFilepath = os.path.join( outputFolder, filepath.replace( '/', '_' ).replace( '\\', '_' ) + '.png' )
        if os.path.exists( pixmapFilepath ) and os.stat( pixmapFilepath )[ stat.ST_MTIME ] > os.stat( filepath )[ stat.ST_MTIME ]:
            pixmap = QPixmap( pixmapFilepath )
            print 'exists'
        else:
            f = open( filepath, 'rt' ); lines = f.readlines(); f.close()
            if lines:
                ##pixmap = QPixmap( max( map( lambda line: len( line.replace( '\t', ' ' * tabWidth ).rstrip() ), lines ) ), lineHeight * len( lines ), 1 )
                pixmap = QPixmap( columnWidth, lineHeight * len( lines ), 1 )
                painter = QPainter()
                painter.begin( pixmap )
                painter.eraseRect( pixmap.rect() )
                y = 1
                for line in lines:
                    x = 0
                    for char in line.rstrip():
                        if char == '\t':
                            x += tabWidth
                        else:
                            if char != ' ':
                                painter.drawPoint( x, y )
                            x += 1
                    y += lineHeight
                painter.end()
                if not os.path.exists( outputFolder ):
                    os.mkdir( outputFolder )
                pixmap.save( pixmapFilepath, 'PNG' )
                print 'rendered'
        pixmapMap[ filepath ] = pixmap.width(), pixmap.height()
    else:
        print 'skipped'

def makeDot():
    f = open( os.path.join( outputFolder, '_graphviz.dot' ), 'wt' )
    f.write( '''digraph G {
bgcolor=grey95
''' )
    for pixmap in pixmapMap:
        width, height = pixmapMap[ pixmap ]
        width /= 95.95
        height /= 95.956
        node = os.path.splitext( pixmap )[ 0 ]
        f.write( '%s [label="", shape=custom, shapefile="file://%s.png", width=%s, height=%s];\n' % ( node, pixmap.replace( '/', '_' ).replace( '\\', '_' ), width, height ) )
    f.write( '}\n' )
    f.close()

def compose():
    columns = 11
    margin = 40
    width = columns * columnWidth + ( columns - 1 ) * margin + 4* margin
    height = width * 1.4142 + 1
    pixmap = QPixmap( width, height, 32 )
    painter = QPainter()
    painter.begin( pixmap )
    painter.eraseRect( pixmap.rect() )
    painter.fillRect( 0, 0, width, height, QBrush( QColor( 0xEE, 0xEE, 0xEE ) ) )
    painter.setFont( QFont( 'Calibri', 59, QFont.Bold ) )
    painter.drawText( margin * 2, margin * 3, 'SilF4ware Metrics: %s Lines of Code in %s Source Files' % ( sum( map( lambda value: value[ 1 ], pixmapMap.values() ) ) / lineHeight, len( pixmapMap ) ) )
    painter.setFont( QFont( 'Tahoma', 8 ) )
    pixmapList = list( pixmapMap.keys() )
    pixmapList.sort( key = lambda a: pixmapMap[ a ][ 1 ] )
    pixmapList.reverse()
    x = margin
    widest = 0
    while len( pixmapList ) > 0:
        x += widest + margin
        y = margin * 5
        widest = 0
        i = 0
        while i < len( pixmapList ):
            filepath = pixmapList[ i ]
            w, h =pixmapMap[ filepath ]
            if y + h < height - margin * 2:
                painter.drawText( x, y - 6, '%s, %s' % ( filepath, h / lineHeight ) )
                painter.drawPixmap( x, y, QPixmap( os.path.join( outputFolder, filepath.replace( '/', '_' ).replace( '\\', '_' ) + '.png' ) ) )
                painter.drawRect( x - 1, y - 1, w + 2, h + 2 )
                y += h + margin
                pixmapList.pop( i )
                if w > widest:
                    widest = w
            else:
                i += 1
    if drawVersion:
        painter.drawText( x, height - margin * 2, 'Version %s, Revision %s' % ( versioninfo.applicationVersionString, versioninfo.svnRevisionNumber ) )
    painter.end()
    # pixmap.save( os.path.join( outputFolder, '_metrics.png' ), 'PNG' )
    pixmap.save( '_birdseyeview.png', 'PNG' )

app = QApplication( sys.argv )
for filepath in os.listdir( '.' ):
    print filepath,
    render( filepath )
##makeDot() # dot -Tpng _graphviz.dot -o _graphviz.png
compose()
