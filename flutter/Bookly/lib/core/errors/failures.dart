import 'package:dio/dio.dart';

abstract class Failure {
  final String errMessage;
  Failure(
    this.errMessage,
  );
}

class ServerFailure extends Failure {
  ServerFailure(super.errMessage);

  factory ServerFailure.fromDioError(DioException dioError) {
    switch (dioError.type) {
      case DioExceptionType.connectionTimeout:
        return ServerFailure('Connection timeout with api server');
      case DioExceptionType.sendTimeout:
        return ServerFailure('Sending timeout with api server');
      case DioExceptionType.receiveTimeout:
        return ServerFailure('Reciveing timeout with api server');
      case DioExceptionType.badCertificate:
      case DioExceptionType.badResponse:
        return ServerFailure.fromResponse(
            dioError.response!.statusCode!, dioError.response!.data);
      case DioExceptionType.cancel:
        return ServerFailure('Request to api server was canceled');
      case DioExceptionType.connectionError:
      case DioExceptionType.unknown:
        if (dioError.message!.contains('SocketException')) {
          return ServerFailure('No Internet connection');
        }

        return ServerFailure('Unexpected Error, Please try again!');
      default:
        return ServerFailure('Oops! There was an error,please try again');
    }
  }

  factory ServerFailure.fromResponse(int statusCode, dynamic response) {
    if (statusCode == 400 || statusCode == 401 || statusCode == 403) {
      return ServerFailure(response['error']['message']);
    } else if (statusCode == 404) {
      return ServerFailure('Request not found, please try again later');
    } else if (statusCode == 500) {
      return ServerFailure('Internal server error, please try again later');
    } else {
      return ServerFailure('Oops! There was an error,please try again');
    }
  }
}
