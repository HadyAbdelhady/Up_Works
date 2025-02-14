import 'package:bookly/Features/home/presentation/manager/feature_books_cubit/featured_books_cubit.dart';
import 'package:bookly/core/widgets/custom_error_widger.dart';
import 'package:bookly/core/widgets/custom_loading_indicator.dart';
import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'custom_book_image.dart';

class FeaturedBooksListView extends StatelessWidget {
  const FeaturedBooksListView({super.key});

  @override
  Widget build(BuildContext context) {
    return BlocBuilder<FeaturedBooksCubit, FeaturedBooksState>(
      builder: (context, state) {
        if (state is FeaturedBooksSuccess) {
          return Padding(
            padding: const EdgeInsets.only(
              left: 8,
              right: 8,
              bottom: 16,
            ),
            child: SizedBox(
              height: MediaQuery.of(context).size.height * 0.25,
              child: ListView.builder(
                padding: EdgeInsets.zero,
                physics: const BouncingScrollPhysics(),
                scrollDirection: Axis.horizontal,
                itemCount: state.books.length,
                itemBuilder: (BuildContext context, int index) {
                  return  CustomBookImage(
                    imageUrl: state.books[index].volumeInfo.imageLinks?.thumbnail ??'',
                  );
                },
              ),
            ),
          );
        } else if (state is FeaturedBooksFailure) {
          return CustomErrorWidget(errMessage: state.errMessage);
        } else {
          return const CustomLoadingIndicator();
        }
      },
    );
  }
}
